# Range 관측 유효성 · FFT1-150

설치된 ROS 2 Jazzy `nav2_costmap_2d::RangeSensorLayer`를 상속한다. 원본 알고리즘·라이브러리·라이선스는
수정하거나 복제하지 않는다. 한 거리와 감지각을 원본 확률 센서 모델로 지도에 반영한다.

추가 동작은 두 가지다. 유효한 유한 거리/감지각과 신선한 timestamp(-0.1..0.3초)만 입력하고,
원본이 TF 변환까지 처리한 새 관측이 있으면 timeout 후 `current` 상태를 회복한다.
센서가 다시 끊기면 기존 `no_readings_timeout=0.5` 검사를 그대로 적용한다.
원본의 cost 합성은 유지하므로 초음파의 빈 공간 관측으로 지도/라이다의 장애물을 지우지 않는다.

현재는 smartcart **시뮬레이션에만** 연결한다. 실물의 범위·TF·감지각·최대 거리 무반환 규약 확인 전에는 연결하지 않는다.
실제 Nav2 controller 시험에서 첫 관측 지연 후 회복과 관측 중단/오래된 값/NaN 시 주행 실패를 검사한다.

원본: [Nav2 Jazzy RangeSensorLayer](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/plugins/range_sensor_layer.cpp),
[Range 설정](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/range/).
