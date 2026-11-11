# 카트 패드 웹

- 스마트카트(Pinky3) 옆 태블릿에서 띄우는 고객 화면이다(SR-20 패드 GUI 대체). 구현 위치는 `src/`, workspace 이름은 `@roport/pad`다.
- `services/roport-web/`에서 `npm run typecheck --workspace @roport/pad`, `npm run build --workspace @roport/pad`로 검사한다.
- 현재는 `src/api.ts`의 데모 데이터로만 동작한다. 서버 연동 시 `api.ts`만 실제 호출로 바꾸고 화면에서 직접 fetch하지 않는다.
- 인도 승인은 서비스 서버가 판단한다(SR-18-2). 패드는 QR 인식 결과만으로 승인 화면을 띄우지 않는다.
- 중지·재개·안내 시작 버튼은 로봇 명령으로 이어질 수 있다. 화면 진입·자동 새로고침으로 명령을 보내지 않고, 접수와 실제 로봇 상태를 구분해 표시한다.
