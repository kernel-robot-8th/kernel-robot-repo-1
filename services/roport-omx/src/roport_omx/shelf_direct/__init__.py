"""shelf-01 OMX1·OMX2 양팔을 OMX 제어판(Panel) 없이 Host가 직접 구동하는 실행부.

OMX3(``task/omx3``)와 같은 구조: Executor → Runner → Robot·Policy 포트 → UDP 추론(infer_link RPI1).
OMX3와 다른 점은 두 팔을 한 모델(state·action 12개, 카메라 4대)로 동시에 움직인다는 것과,
현장에서 Panel이 하던 안전 시작·착지와 async 추론 규칙을 그대로 옮겼다는 것이다.

import만으로 장치를 열지 않는다. lerobot·cv2·numpy는 실제로 쓰는 함수 안에서 불러온다.
"""
