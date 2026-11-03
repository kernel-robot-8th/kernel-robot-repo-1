# roport-ai-server

RO:PORT · GATE CREW — 모델 로딩·추론 API·모델 버전 관리

## 기술 스택

Python 3.12 / FastAPI / Uvicorn / uv / pytest / Ruff. 모델 런타임은 PyTorch 기준이며 모델·CUDA·드라이버 조합 확정 후 추가합니다.

## 현재 범위

FastAPI 시작 구조, 분담 정책 라우터, 로컬 ACT checkpoint/저장 processor 로딩과 관측별 UDP 영상 resolver를 제공합니다. 기본 검사는 fake 모델·합성 영상·loopback입니다. 실제 체크포인트의 CPU/GPU 실행은 별도 확인이 필요합니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 관련 문서

이 디렉토리는 단일 저장소의 구성요소입니다. 아래 명령은 이 디렉토리에서 실행합니다.

- [프로젝트 개요](https://github.com/kernel-robot-8th/kernel-robot-repo-1)
- [요구사항·설계 문서](../../docs/README.md)
- [Jira](https://fp-team1.atlassian.net/jira/projects/FFT1)

PR에 관련 Jira 키, 변경 내용, 검증 결과와 미검증 항목을 기록합니다.
실로봇 적용 전 장비·설정·캘리브레이션, 안전 정지·복구 절차를 확인합니다.

## 개발 방법

```bash
python3 -m pip install uv==0.13.5
uv sync --locked
uv run ruff check .
uv run pytest
uv build
```

API 실행: `uv run uvicorn app:app --app-dir src --host 127.0.0.1 --port 8000`

`/healthz`는 프로세스 상태만 확인합니다. 로봇·DB·모델 준비 상태를 뜻하지 않습니다.

## 분담 정책 후보 · source/mock

`dedicated_policy.PolicyRouter(policies, metadata)`에 작업·단계별 provider와 모델/feature/보정 메타데이터를 등록합니다. provider는 `reset()`과 `infer(observation)`을 구현하고, 관절 5개+그리퍼 1개의 위치 후보 묶음을 반환합니다. 이는 LeRobot ACT 연결점이며 등록만으로 학습된 ACT가 준비되지는 않습니다. `/readyz`는 계속 미준비를 반환합니다.

- wire profile: `omx.dedicated/1`, ISO 8601 offset 시각, 최대 60,000-byte strict JSON. 중복 키·비유한 값·잘못된 feature 순서·이전 실행/단계·만료된 관측은 거절합니다.
- `handle(request, now=UTC_epoch_seconds)`와 `handle_datagram(bytes, now=...)`는 직렬로 호출합니다. 단계 revision이 바뀌면 reset을 다시 확인하며 미등록 모델·provider 오류는 `inference.error`입니다.
- `dedicated_udp.serve_once(socket, router, allowed_peer=..., clock=...)`는 호출자가 준비한 UDP 소켓에서 지정 peer의 요청 1개를 처리합니다. import·HTTP 서버 시작만으로 소켓을 열지 않습니다. `allowed_peer`는 IPv4 `(host, port)` 2-tuple 전용입니다(IPv6 미지원). 다른 peer의 패킷은 응답 없이 버리고 경고 로그를 남긴 뒤 `"unexpected_policy_peer"`를 반환하며, 처리한 경우 `None`을 반환하므로 호출자는 예외 없이 루프를 계속합니다. peer 제한은 암호학적 인증이 아니며 현장 네트워크 적용 전 별도 검증이 필요합니다.
- 동기 provider를 강제 중단하지 않습니다. 반환 후 deadline이 지난 후보는 폐기하고 Host도 자체 timeout으로 보류해야 합니다. 모터 구동·완료·Pinky 출발은 AI의 권한이 아닙니다.

## ACT와 관측별 영상

`act_provider.ActProvider(checkpoint, metadata, image_resolver)`를 `PolicyRouter`의 provider로 등록합니다. checkpoint는 로컬 디렉터리이고 `config.json`, `model.safetensors`, `policy_preprocessor.json`, `policy_postprocessor.json`, 각 processor state와 `metadata.json`이 필요합니다. 등록 metadata와 operation·담당 팔·phase·모델/묶음/schema/보정 버전·feature 순서/단위·카메라·chunk 설정이 같아야 합니다. 장치 선택은 명시한 `cpu` 또는 `cuda`입니다.

LeRobot v0.6.1 환경에서 지연 로드하므로 코어 import·CI가 torch/CUDA나 가중치를 다운로드하지 않습니다. 완성 checkpoint의 backbone을 쓰며 ImageNet 다운로드도 차단합니다. 저장한 preprocessor로 RGB/관절을 처리하고 postprocessor로 action을 역정규화합니다. `reset()`은 policy와 두 processor를 모두 초기화합니다. `infer()`는 현재 관측의 유한한 6차원 후보를 `n_action_steps`만큼 반환하며 오류 뒤에는 reset이 필요합니다. Temporal ensemble checkpoint는 현재 지원하지 않습니다.

`metadata.cameras`는 wire camera key별 `{shape: [3,H,W], fps: ...}`입니다. 데이터셋의 HWC `observation.images.<camera>`와 명시적으로 변환합니다. 팔 단위는 `range_m100_100` 또는 `degrees`, 그리퍼는 `range_0_100`이며 보정·방향·ROS m/rad와 자동 혼용하지 않습니다. 학습 metadata 형식과 실행 CLI는 [OMX README](../roport-omx/README.md)의 패널 실행 연동 항목을 참고합니다.

`observations.ObservationImageResolver(streams, timeout=..., max_inflight=..., max_bytes=..., max_frames=..., max_age_ns=..., max_timestamp_skew_ns=...)`에 allowlist와 모든 한계를 명시합니다. `streams`는 `{(source,camera,stream): ((peer_ip,peer_port), UUID(session))}`입니다. 수신 socket loop에서 `feed(packet,peer)`를 호출하고, provider에는 resolver 자체를 전달합니다. feed와 infer는 같은 직렬 소비 흐름에서 호출합니다.

RPV1은 관제 portable protocol/receiver를 재사용하며 1200-byte 분할·최대 1 MiB JPEG·CRC를 유지합니다. 관측의 `images[camera]`와 source/camera/stream/session/frame_id/captured_ns/observation_group이 전부 같은 완성 프레임만 반환합니다. `observation.captured_at`도 원 관절 관측 시각이어야 합니다. 다른 peer/session/group·늦은 영상·과대 cache를 거절하고 과거 프레임으로 대신하지 않습니다. 정책 JSON에는 영상 bytes나 base64를 넣지 않습니다.

metadata에 카메라가 등록된 요청은 router가 camera manifest와 envelope의 captured_at 일치를 먼저 검사합니다. 잘못된 입력은 `invalid_observation`, 유효한 현재 영상이 아직 없으면 `input_not_ready`, 모델 실행 오류는 `inference_failed`입니다. 미등록/미준비 provider를 HTTP `/readyz` 성공으로 숨기지 않으며 실제 모델·영상·Host 연결을 확인하기 전에는 기존 503을 유지합니다.

CPU/GPU replay·실제 JPEG 디코딩·모델 성능·현장 지연/유실·실장비 인수는 별도 검증입니다. 기본 CI는 실제 가중치나 현장 영상을 사용하지 않습니다.
