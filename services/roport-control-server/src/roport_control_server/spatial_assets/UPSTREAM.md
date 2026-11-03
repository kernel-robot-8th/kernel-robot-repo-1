# 로컬 표시 자원

- Three.js: https://github.com/mrdoob/three.js/tree/r187 (npm 0.187.0), MIT. 서비스의 `package-lock.json`으로 버전·다운로드 무결성을 고정한다. `npm ci --ignore-scripts` 후 `npm run prepare:assets`가 필요한 build 파일(폴더명만 `esm`)·loader/control/utils·LICENSE를 SHA-256 검증 후 `vendor/three`에 복사한다. 이 생성 디렉터리는 Git에서 제외하고 wheel/sdist에 포함한다. 런타임 CDN 요청 없음.
- Pinky: 출처·커밋의 기준은 [Pinky 원본 반입 기록](../../../../roport-pinky/src/UPSTREAM.md)이다. 생성기가 해당 커밋을 읽고 `models/sources.json`에 기록하며, 테스트가 둘의 일치를 검사한다. Apache-2.0의 로컬 `pinky_description`에서 파생하며, 설치 패키지에도 출처 메타데이터와 `models/pinky/LICENSE`를 보존한다.
- OMX-F: https://github.com/ROBOTIS-GIT/open_manipulator/tree/b83daec7b98c08d00adb856d7e31f2b0065d3be9, Apache-2.0. 이 커밋의 `omx_f.urdf`와 visual STL에서 파생. 장치 드라이버를 실행하거나 변경하지 않는다.
- 모델마다 URDF, joints.json, LICENSE를 함께 보존한다. Pinky는 자동 경량화 시 표면이 손상되어 원본 visual의 면 구조를 유지한다(총 287,030개 삼각형). OMX는 visual당 8,000면을 목표로 단순화한다. 모두 중립색 표시용 자원이다. 재생성: `tools/prepare_display_models.py` 상단의 고정 버전 명령. 단위 m, ROS Z-up, 관절 rad, 원점 RPY 및 mimic 유지. 실제 기기의 영점·관절 방향·Pinky 화면/카메라 각도는 미검증.
- `pinky-pro.webp`, `omx-f.webp`: 포함된 Apache-2.0 파생 GLB를 `tools/render_display_thumbnails.py`로 직접 렌더링한 이미지다. 공식 사진이나 사진을 참조한 AI 생성 이미지를 사용하지 않는다. 원본·LICENSE는 각 모델 폴더와 `models/sources.json`, 렌더 입력 GLB의 경로·SHA-256은 `thumbnails.json`에 보존한다. 재생성 명령은 스크립트 상단 참조. Three.js 0.187.0, 512×512 투명 WebP, URDF 기본 자세, Z-up 3/4 시점·직교 투영으로 생성한다. GPU·Qt 차이로 픽셀은 달라질 수 있다.
- `maps/demo.*`: 이번 UI 확인용 예시 지도. 실제 창고 구조 또는 팀원 airport_map 복제본이 아니다. 지도 벽 높이 0.35 m는 표시용 가정이다.

- GLB는 현재 Git에 직접 보관한다(Pinky 약 6 MiB, OMX 약 1.1 MiB). 원본 모델·변환 방식이 바뀔 때만 재생성하며 UI 변경이나 빌드마다 다시 만들지 않는다. 누적 용량이 부담이 될 때 LFS 도입을 별도 검토한다.
- 실제 OMX 연결 시 텔레메트리의 관절 이름·단위·영점·부호를 이 URDF의 관절(rad)과 대조하는 매핑이 필요하다. LeRobot의 도/정규화 값을 그대로 적용하지 않는다. 이 패키지는 표시 자원만 제공한다.

`checksums.json`은 포함된 외부 자원과 파생 모델의 SHA-256이다. 자원이 변경되면 출처와 영향받은 항목만 함께 갱신한다. URDF 상단에는 RO:PORT 파생물 표시를 보존한다.
