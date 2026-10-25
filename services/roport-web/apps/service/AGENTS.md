# 서비스 웹

- 고객 화면 범위다. 구현 위치는 `src/`, workspace 이름은 `@roport/service`다.
- `services/roport-web/`에서 `npm run typecheck --workspace @roport/service`, `npm run build --workspace @roport/service`로 검사한다.
- 고객 예약·주문·픽업 상태는 서버 계약에 맞춘다. 도착·직원 확인·고객 인도 완료를 구분한다.
- 어드민 전용 기능·자격 증명을 고객 앱에 포함하지 않는다. 미정인 API는 구현된 서버 기능처럼 표시하지 않는다.
