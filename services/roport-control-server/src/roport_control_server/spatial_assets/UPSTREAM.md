# 로컬 표시 자원

- Three.js: https://github.com/mrdoob/three.js/tree/r187 (npm 0.187.0), MIT. `vendor/three`에 공식 build 파일(폴더명만 `esm`)과 필요한 loader/control/utils를 원본 그대로 보관한다. 런타임 CDN 요청 없음.
- Pinky: https://github.com/pinklab-art/pinky_pro/tree/014a09f289e6988894fbffde2624fdb1e257815b, Apache-2.0. 저장소의 동일 버전 `pinky_description`에서 파생.
- OMX-F: https://github.com/ROBOTIS-GIT/open_manipulator/tree/b83daec7b98c08d00adb856d7e31f2b0065d3be9, Apache-2.0. 이 커밋의 `omx_f.urdf`와 visual STL에서 파생. 장치 드라이버를 실행하거나 변경하지 않는다.
- 모델마다 URDF, joints.json, LICENSE를 함께 보존한다. Pinky는 자동 경량화 시 표면이 손상되어 원본 visual의 면 구조를 유지한다(총 287,030개 삼각형). OMX는 visual당 8,000면을 목표로 단순화한다. 모두 중립색 표시용 자원이다. 재생성: `tools/prepare_display_models.py` 상단의 고정 버전 명령. 단위 m, ROS Z-up, 관절 rad, 원점 RPY 및 mimic 유지. 실제 기기의 영점·관절 방향·Pinky 화면/카메라 각도는 미검증.
- `pinky-pro.webp`, `omx-f.webp`: 사용자가 선택한 앞선 UI 시안의 단순화 이미지. 공식 제품 사진으로 표시하지 않는다.
- `maps/demo.*`: 이번 UI 확인용 예시 지도. 실제 창고 구조 또는 팀원 airport_map 복제본이 아니다. 지도 벽 높이 0.35 m는 표시용 가정이다.

`checksums.json`은 포함된 외부 자원과 파생 모델의 SHA-256이다. 변경 시 출처와 함께 갱신한다.
