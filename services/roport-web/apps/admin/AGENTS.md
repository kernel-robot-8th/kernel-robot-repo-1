# 어드민 웹

- 직원·관리자 화면 범위다. 구현 위치는 `src/`, workspace 이름은 `@roport/admin`이다.
- `services/roport-web/`에서 `npm run typecheck --workspace @roport/admin`, `npm run build --workspace @roport/admin`으로 검사한다.
- 직원·관리자 권한과 작업 상태를 서버 계약에 맞춘다. UI에서 버튼을 숨기는 것을 서버 권한 검사로 취급하지 않는다.
- 로봇 작업 지시·정지·복구 UI는 접수와 실제 장비 상태를 구분한다. 화면 진입이나 자동 새로고침으로 장비 명령을 보내지 않는다.
