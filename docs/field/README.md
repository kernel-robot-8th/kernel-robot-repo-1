# Fleet 현장 배포 준비와 복구

저장소의 `services/roport-control-server`에서 실행한다. 다음 명령은 새 디렉터리에 배포 자료를 만들고 설정 오류를 JSON으로 기록한다.

```bash
uv run python tools/prepare_field.py --output /tmp/field-handoff
(cd /tmp/field-handoff && sha256sum -c SHA256SUMS)
uv run python tools/check_field.py --bundle /tmp/field-handoff --targets pinky1 omx3 --output /tmp/field-check.json
```

- 원본 묶음을 보관하고 작업 사본을 사용한다. Pinky에는 해당 슬롯의 `trial.json`, `pinky_mqtt/`, 지도와 필요한 ROS 패키지를 전달한다. OMX에는 `omx/`를 전달한다. 기존 보정·환경·journal은 기체에 유지한다.
- 새 설정은 현장 확인 전 상태다. 실제 지도와 복귀 목표 `n12_goal`, 기체 배정, 모델·보정 버전을 확인한 뒤 작업 사본의 관제 및 대상 Pinky 설정에 현장 확인을 기록한다. 초기 점검은 미확정 항목을 표시하고 종료 코드 1을 반환한다.
- 이미 실행 중인 기체 상태를 관찰할 때는 브로커 환경을 지정하고 점검 명령에 `--listen --seconds 5`를 추가한다. `--nav2 pinky1`은 해당 기체의 Nav2 및 위치 추정 준비도 확인한다. 이 도구는 동작 명령을 발행하지 않는다.
- 점검 실패 시 보고서의 대상·원인을 확인한다. 이전 배포로 돌아갈 때는 보관한 원본과 해당 시점의 지도·코드 해시를 대조하고 미완료 명령의 journal을 보존한다. 재시작 후 HOLD 해제 및 재출발은 운영자가 별도로 결정한다.
