"""OMX 3 Host: 상차 요청 접수·중복 처리·실행·결과 보고.

MQTT 송수신과 envelope 검증(시각·만료·서명)은 관제가 인계할 공통 모듈이 맡는다(9단계).
이 클래스는 그 모듈이 넘겨준 요청(dict)을 받아 ``publish(type, request, payload)``로 응답만 만든다.
이벤트 이름은 Tech Spec v9 제안 ``load.accepted / progress / completed / failed / rejected``.

이력은 JSON Lines 파일에 fsync해서 남긴다. 재시작 때 끝나지 않은 실행이 있으면 RECONCILING으로
새 실행을 막는다. 사람이 팔·물품을 확인한 뒤 ``clear_reconciling(key)``로 푼다.
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import threading
import uuid
from collections.abc import Callable
from pathlib import Path

from .contract import ErrorCode, Outcome, Phase
from .runner import Runner
from .settings import Omx3Settings
from .target import InvalidTarget, resolve_marker_id

Publish = Callable[[str, dict, dict], None]
log = logging.getLogger(__name__)
PROGRESS = (Phase.HOMING, Phase.RUNNING, Phase.VERIFY)


def _key(request: dict) -> str:
    """같은 명령의 재전송을 알아보는 키(관제 envelope의 command_id·execution_id)."""
    return f"{request.get('command_id')}/{request.get('execution_id')}"


def _fingerprint(request: dict) -> str:
    body = json.dumps(request.get("payload"), sort_keys=True, ensure_ascii=False)
    return hashlib.sha256(body.encode()).hexdigest()


class Omx3Host:
    def __init__(
        self,
        runner: Runner,
        publish: Publish,
        settings: Omx3Settings | None = None,
        journal_path: str | os.PathLike | None = None,
    ):
        self.runner, self.publish = runner, publish
        self.settings = settings or runner.settings
        self.boot_id = (
            uuid.uuid4().hex
        )  # 재시작을 관제가 알 수 있게 모든 이벤트에 붙인다
        self.journal = Path(journal_path) if journal_path else None
        self._lock = threading.RLock()  # 요청 처리와 실행 스레드의 이벤트 기록을 직렬화
        # shortcut: 기록을 메모리에 계속 쌓는다(시험 규모). 장기 운영 때 오래된 키 정리 추가
        self._records: dict[str, dict] = {}
        self._active: str | None = None
        self._abort = threading.Event()
        self._thread: threading.Thread | None = None
        self.reconciling: list[str] = []
        self._load_journal()

    # ------------------------------------------------------------ 요청 처리
    def handle_load(self, request: dict) -> None:
        s = self.settings
        key, fp = _key(request), _fingerprint(request)
        with self._lock:
            known = self._records.get(key)
            if known is not None:
                if (
                    known["fp"] == fp
                ):  # 재전송: 다시 실행하지 않고 마지막 이벤트만 다시 보낸다
                    self.publish(known["last_type"], request, known["last_payload"])
                else:
                    self._reject(request, ErrorCode.IDEMPOTENCY_CONFLICT, record=False)
                return
            if self.reconciling:
                return self._reject(request, ErrorCode.RECONCILING, record=False)
            if self._active is not None:
                return self._reject(request, ErrorCode.BUSY, record=False)
            try:
                marker_id = resolve_marker_id(
                    request.get("payload"), s.id_field, s.package_ids
                )
            except InvalidTarget as exc:
                return self._reject(request, ErrorCode.INVALID_TARGET, str(exc), fp=fp)
            code = self.runner.precheck(marker_id)
            if code is not None:
                return self._reject(request, code, marker_id=marker_id, fp=fp)
            self._records[key] = {"fp": fp, "done": False}
            self._active = key
            self._emit(request, "load.accepted", {"marker_id": marker_id})
            self._abort.clear()
            self._thread = threading.Thread(
                target=self._execute, args=(request, marker_id), daemon=True
            )
            self._thread.start()

    def abort(self) -> None:
        """실행 중인 명령을 멈춘다(runner가 CANCELED로 끝내고 자세를 유지한다)."""
        self._abort.set()

    def wait(self, timeout: float | None = None) -> None:
        thread = self._thread
        if thread is not None:
            thread.join(timeout)

    def status(self) -> dict:
        with self._lock:
            robot = self.runner.robot.ready()
            return {
                "boot_id": self.boot_id,
                "ready": robot is None
                and self._active is None
                and not self.reconciling,
                "robot": robot or "ok",
                "active": self._active,
                "reconciling": list(self.reconciling),
            }

    def clear_reconciling(self, key: str) -> None:
        with self._lock:
            if key in self.reconciling:
                self.reconciling.remove(key)
                self._write({"key": key, "event": "reconciled"})

    # ------------------------------------------------------------ 내부
    def _execute(self, request: dict, marker_id: int) -> None:
        def on_phase(phase: Phase, info: dict) -> None:
            if phase in PROGRESS:
                with self._lock:
                    self._emit(request, "load.progress", {"phase": str(phase)})

        try:
            outcome = self.runner.run(marker_id, self._abort, on_phase)
        except Exception as exc:  # 실행 오류도 실패로 보고(log.exception으로 남김)
            log.exception("run failed")
            outcome = Outcome(
                Phase.FAILED, ErrorCode.ROBOT_ERROR, marker_id, detail=repr(exc)
            )
        kind = "load.completed" if outcome.success else "load.failed"
        with self._lock:
            try:
                self._emit(request, kind, outcome.to_dict(), done=True)
            finally:  # 보고가 실패해도 다음 명령을 막지 않는다(기록은 이미 남았다)
                self._active = None

    def _reject(
        self,
        request: dict,
        code: ErrorCode,
        detail: str = "",
        marker_id: int | None = None,
        fp: str | None = None,
        record: bool = True,
    ) -> None:
        payload = {
            "phase": str(Phase.REJECTED),
            "error_code": str(code),
            "marker_id": marker_id,
            "detail": detail,
        }
        if (
            not record
        ):  # BUSY 등은 키를 남기지 않는다: 나중에 같은 명령을 다시 받을 수 있다
            self.publish("load.rejected", request, self._stamp(payload))
            return
        self._records[_key(request)] = {"fp": fp or _fingerprint(request), "done": True}
        self._emit(request, "load.rejected", payload, done=True)

    def _emit(
        self, request: dict, kind: str, payload: dict, done: bool = False
    ) -> None:
        key, payload = _key(request), self._stamp(payload)
        record = self._records[key]
        record.update(last_type=kind, last_payload=payload, done=done or record["done"])
        # 보내기 전에 기록한다: 보낸 뒤 죽어도 재시작 때 상태를 안다
        self._write(
            {
                "key": key,
                "fp": record["fp"],
                "type": kind,
                "payload": payload,
                "done": record["done"],
            }
        )
        self.publish(kind, request, payload)

    def _stamp(self, payload: dict) -> dict:
        return {"boot_id": self.boot_id, **payload}

    def _write(self, entry: dict) -> None:
        if self.journal is None:
            return
        self._write_raw(json.dumps(entry, ensure_ascii=False) + "\n")

    def _write_raw(self, text: str) -> None:
        self.journal.parent.mkdir(parents=True, exist_ok=True)
        with self.journal.open("a", encoding="utf-8") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())

    def _load_journal(self) -> None:
        if self.journal is None or not self.journal.exists():
            return
        reconciled = set()
        text = self.journal.read_text(encoding="utf-8")
        if text and not text.endswith("\n"):
            self._write_raw(
                "\n"
            )  # 끊긴 줄을 닫아 다음 기록이 그 줄에 이어 붙지 않게 한다
        for n, line in enumerate(text.splitlines(), 1):
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:
                # 쓰는 중 전원이 꺼진 줄. 건너뛰면 끝나지 않은 실행으로 남아 RECONCILING(안전한 쪽)
                log.warning("journal line %d unreadable, skipped", n)
                continue
            if entry.get("event") == "reconciled":
                reconciled.add(entry["key"])
                continue
            self._records[entry["key"]] = {
                "fp": entry["fp"],
                "done": entry["done"],
                "last_type": entry["type"],
                "last_payload": entry["payload"],
            }
        self.reconciling = [
            k for k, r in self._records.items() if not r["done"] and k not in reconciled
        ]
