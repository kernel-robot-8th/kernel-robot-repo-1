# roport-web

RO:PORT · GATE CREW — 고객 서비스 웹·어드민 웹·카트 패드 웹

## 기술 스택

Node.js 24 / React 19 / TypeScript 7 / Vite 8 / npm workspaces. apps/service, apps/admin, apps/pad를 별도 빌드합니다.

통합관제 3D 웹 UI는 [roport-control-server](../roport-control-server/README.md)에서 FastAPI와 함께 제공합니다. 이 디렉터리의 고객·업무 어드민 앱과 별도로 관리하며 PyQt 탭을 사용하지 않습니다.

## 현재 범위

고객 앱은 서비스 서버의 주문·예약과 관제 진행 상태를 조회합니다. 패드 기본 모드는 데모이며 `VITE_PAD_API=server`에서는 QR/예약번호 확인과 수령 기록을 실제 서비스 서버에 요청합니다. 수령 성공에는 관제의 해당 카트 도착 증거가 필요합니다. `VITE_PAD_FOLLOW=1`이면 현재 카트가 지원하는 마커·외형 추종을 수령과 함께 선택합니다. 외형 촬영·대상 등록 후 첫 출발은 관제 승인으로 진행하고 정지·재개·복귀와 상태 복원을 서비스 API로 처리합니다. 실제 Pinky 실행부 연결·운영 인증은 별도이며 [추종 연동 인수인계](../roport-control-server/docs/follow-modes-handover.md)를 따릅니다. 게이트 안내는 기존 데모 범위입니다.
[고객 웹 → 서비스 → MQTT 가상 로봇 → 패드 수령·복귀 통합 시험](../roport-control-server/docs/service-web-local-test.md)은 현재 로컬 코드 전체를 한 명령으로 실행합니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 관련 문서

이 디렉토리는 단일 저장소의 구성요소입니다. 아래 명령은 이 디렉토리에서 실행합니다.

- [프로젝트 개요](https://github.com/kernel-robot-8th/kernel-robot-repo-1)
- [요구사항·설계 문서](../../docs/README.md)
- [Jira](https://fp-team1.atlassian.net/jira/projects/FFT1)

PR에 관련 Jira 키, 변경 내용, 검증 결과와 미검증 항목을 기록합니다.
실로봇 적용 전 장비·설정·캘리브레이션, 안전 정지·복구 절차를 확인합니다.

## 개발 방법

```bash
npm ci
npm run typecheck
npm run build
npm run dev --workspace @roport/service
# 별도 터미널
npm run dev --workspace @roport/admin -- --port 5174
npm run dev --workspace @roport/pad -- --port 5175
```

카트 패드의 예약 확인·수령 기록을 로컬 서비스 서버(`127.0.0.1:8080`, `SERVICE_DEV_CART=true`)에 붙여 보려면 `VITE_PAD_API=server npm run dev --workspace @roport/pad -- --port 5175`로 실행합니다. 개발 서버가 `/cart-api`를 서비스 서버로 넘기며, 카트 ID는 `VITE_PAD_CART_ID`(기본 `pinky3`)입니다. 배포 nginx에는 아직 이 전달이 없어 배포 빌드는 데모로 둡니다.

기본 패드의 **짐 운반·추종**은 기존 마커 카드 FOLLOW 흐름입니다. 외형 선택을 시험할 때만 `VITE_PAD_SHOPPING_DEMO=true npm run build --workspace @roport/pad -- --outDir .cache/dist-shopping-demo`로 별도 빌드하고, 로컬 쇼핑 시험 실행부에서 `apps/pad/.cache/dist-shopping-demo`를 제공합니다. 데모 예약의 서비스 선택 화면에 **외형 추종 단독 시험** 버튼이 따로 뜹니다. 일반 빌드에는 이 진입점이 없고, `VITE_PAD_API=server` 모드에서는 플래그가 있어도 열리지 않습니다.

e2e(`e2e/`, CI `e2e-ci.yml`): 서비스 서버를 위 조건(`SERVICE_DEV_LOGIN=true`, `SERVICE_DEV_CART=true`, 개발 시드 `db/dev/seed.sql` 적용)으로 띄운 뒤 `npx playwright install chromium`(처음 한 번), `npm run test:e2e`로 실행합니다. 패드(5185, 서버 모드)와 고객 웹(5186) 개발 서버는 Playwright가 띄우고, 예약은 고객 API로 만듭니다. 고객 웹 시나리오는 화면이 받은 예약 QR이 카트 확인에서 통과하는지와 만료 후 재발급을 봅니다.

각 앱의 `dist/`를 별도로 빌드할 수 있습니다. 개발 전용 로그인·카트 인증은 공개 배포에 사용할 수 없습니다.
브라우저에 전달하는 환경변수에 비밀키를 넣지 않습니다.

### 쇼핑 모드

카트 패드에서 입국장 물품을 확인하고 **외형 추종 단독 시험 → 카메라 켜기**를 누른다. 이 빌드에서 **짐 운반·추종 · 화면 데모**는 카드 추종 화면을 체험하는 버튼이다. **내 모습 찾기**로 한 장을 고정한 뒤 번호를 누르거나 **직접 선택**으로 몸통을 드래그한다. 검출 박스가 없으면 바로 드래그할 수 있다. 같은 대상과 센서가 확인된 뒤 **따라가기 시작**을 누른다. **잠시 멈추기** 후 같은 대상을 재개하거나 쇼핑을 종료할 수 있다.

이 화면은 Pinky의 로컬 `shopping_follow` 실행부가 제공하는 동일 출처의 `/shopping/` API를 사용한다. 단독 시험은 해당 실행부가 빌드된 패드를 함께 제공한다. 패드 정적 배포와 관제 접수만으로 실물 추종을 시작하지 않는다. [Pinky 쇼핑 실행 방법](../roport-pinky/src/roport_smartcart/README.md#쇼핑-추종)
