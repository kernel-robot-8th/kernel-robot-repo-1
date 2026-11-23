# Upstream: OMX 제어판(Panel)

- Source: https://github.com/tjkoobro-netizen/omx-control-panel (`omx_tools/`)
- Commit: `3edb249` (v3.21.1). 현장 설치본(`~/il_ws/omx_control_panel_v3/omx_tools`)과 같은 내용임을 diff로 확인했다.
- Files: `handover_judge.py`(놓음 판정 1117e), `omx_lift.py`(손끝 들기 1118a), `handover_fk.json`(FK 표·위 카메라·선반 보정)

변경: `omx_lift.py`의 `from handover_judge import ...` 두 곳을 패키지 상대 import로 바꿨다. 다른 내용은 그대로다.
현장 판정과 같은 결과를 내야 하므로 이 파일들은 고치지 않는다. Panel 쪽을 고치면 같은 커밋 기준으로 다시 복사하고 이 기록을 갱신한다.
ruff 검사에서 제외한다(pyproject `extend-exclude`).
