# 카트 패드 웹

- 스마트카트(Pinky3) 옆 태블릿에서 띄우는 고객 화면이다(SR-20 패드 GUI 대체). 구현 위치는 `src/`, workspace 이름은 `@roport/pad`다.
- `services/roport-web/`에서 `npm run typecheck --workspace @roport/pad`, `npm run build --workspace @roport/pad`, `node --test 'apps/pad/src/**/*.test.ts'`로 검사한다.
- 기본은 `src/api.ts`의 데모 데이터로 동작한다. `VITE_PAD_API=server`로 빌드·실행하면 QR·예약번호 확인은 서비스 서버 `POST /cart-api/verifications`(개발 전용 카트 인증 `X-Cart-Id`)를 부르고, 수령은 `POST /cart-api/receipts`로 명시 요청한다. `VITE_PAD_FOLLOW=1`이면 수령과 함께 `none/marker/appearance` 선택을 원자적으로 저장한다. 추종 지원 여부는 `GET /cart-api/follow/capabilities`, 세션 상태는 `GET /cart-api/follow/{receiptId}`로 확인하고 명시 조작만 `/actions`로 요청한다. `FollowSession.tsx`는 서버 상태만 표시하며 단독 시험 `Shopping.tsx`의 직접 시작·heartbeat를 호출하지 않는다. 서버 호출은 `api.ts`에만 두고 화면에서 직접 fetch하지 않는다. 응답 해석은 `src/verify.ts`, 실패 안내 문구는 `src/messages.ts`에 두고 각각의 `*.test.ts`로 검사한다. 추종 브라우저 회귀는 `npx playwright test -c apps/pad/playwright.follow.config.ts`로 검사한다(격리 HTTP fixture, 실로봇 미연결).
- 인도 승인은 서비스 서버가 판단한다(SR-18-2). 패드는 QR 인식 결과만으로 승인 화면을 띄우지 않는다.
- 중지·재개·안내 시작 버튼은 로봇 명령으로 이어질 수 있다. 화면 진입·자동 새로고침으로 명령을 보내지 않고, 접수와 실제 로봇 상태를 구분해 표시한다.
- 화면 상태 전이는 `src/flow.ts`에 둔다. 요청 응답은 보낸 뒤 화면이 바뀌지 않았을 때만 반영하고, 정지·주행 표시는 카트 상태 보고(데모: 모의 상태 입력)로만 바꾼다.
- 탑승 시간 알림 창(`BoardingAlert.tsx`)은 추종 화면(`FollowSession`)에서 패드가 판단해 띄운다. `VITE_PAD_BOARDING_ALERT_S`(초, 기본 없음 = 끔)를 주면 실제 추종(`following`)이 그 시간 이어진 뒤 뜬다. 조회 실패·오래된 상태·HOLD·정지로 바뀌면 창과 로봇 표시를 끄고, 고객이 「확인」으로 닫은 뒤에는 같은 수령에서 다시 띄우지 않는다. 편명·탑승 시각·게이트는 예약 확인 응답 값을 표시만 하고, 탑승 시각 기준 판단은 아직 없다. 창은 명령을 보내지 않는다(게이트 안내 전환 경로가 없어 「확인」만). `VITE_PAD_BOARDING_ROBOT=1`이면 창이 떠 있는 동안 개발 서버 프록시 `/follow-tool`로 같은 PC의 `roport_follow tool --boarding-alert`에 로봇 표시를 부탁한다. 검사: `npx playwright test --config apps/pad/playwright.boarding.config.ts`.
