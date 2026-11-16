# 카트 패드 웹

- 스마트카트(Pinky3) 옆 태블릿에서 띄우는 고객 화면이다(SR-20 패드 GUI 대체). 구현 위치는 `src/`, workspace 이름은 `@roport/pad`다.
- `services/roport-web/`에서 `npm run typecheck --workspace @roport/pad`, `npm run build --workspace @roport/pad`, `node --test 'apps/pad/src/**/*.test.ts'`로 검사한다.
- 기본은 `src/api.ts`의 데모 데이터로 동작한다. `VITE_PAD_API=server`로 빌드·실행하면 예약번호 확인만 서비스 서버 `POST /cart-api/verifications`(개발 전용 카트 인증 `X-Cart-Id`)를 부르고, 나머지 호출은 아직 데모다. 서버 호출은 `api.ts`에만 두고 화면에서 직접 fetch하지 않는다. 응답 해석은 `src/verify.ts`에 두고 `verify.test.ts`로 검사한다.
- 인도 승인은 서비스 서버가 판단한다(SR-18-2). 패드는 QR 인식 결과만으로 승인 화면을 띄우지 않는다.
- 중지·재개·안내 시작 버튼은 로봇 명령으로 이어질 수 있다. 화면 진입·자동 새로고침으로 명령을 보내지 않고, 접수와 실제 로봇 상태를 구분해 표시한다.
- 화면 상태 전이는 `src/flow.ts`에 둔다. 요청 응답은 보낸 뒤 화면이 바뀌지 않았을 때만 반영하고, 정지·주행 표시는 카트 상태 보고(데모: 모의 상태 입력)로만 바꾼다.
