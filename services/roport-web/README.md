# roport-web

RO:PORT · GATE CREW — 고객 서비스 웹·어드민 웹·카트 패드 웹

## 기술 스택

Node.js 24 / React 19 / TypeScript 7 / Vite 8 / npm workspaces. apps/service, apps/admin, apps/pad를 별도 빌드합니다.

통합관제 3D 웹 UI는 [roport-control-server](../roport-control-server/README.md)에서 FastAPI와 함께 제공합니다. 이 디렉터리의 고객·업무 어드민 앱과 별도로 관리하며 PyQt 탭을 사용하지 않습니다.

## 현재 범위

개발 시작용 구조와 CI를 제공합니다. 카트 패드(apps/pad)는 서버 없이 `src/api.ts`의 데모 데이터로 수령 흐름(대기 → QR/예약번호 확인 → 물품 확인 → 입국장 서비스 선택 → 수령 확인 → 복귀)을 보여 주며, 서버·로봇 연동은 하지 않습니다. 로봇 제어·업무 기능·모델 추론은 아직 구현하지 않았습니다.
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

카트 패드의 예약번호 확인을 로컬 서비스 서버(`127.0.0.1:8080`, `SERVICE_DEV_CART=true`)에 붙여 보려면 `VITE_PAD_API=server npm run dev --workspace @roport/pad -- --port 5175`로 실행합니다. 개발 서버가 `/cart-api`를 서비스 서버로 넘기며, 카트 ID는 `VITE_PAD_CART_ID`(기본 `pinky3`)입니다. 배포 nginx에는 아직 이 전달이 없어 배포 빌드는 데모로 둡니다.

기본 패드의 **짐 운반·추종**은 기존 마커 카드 FOLLOW 흐름입니다. 외형 선택을 시험할 때만 `VITE_PAD_SHOPPING_DEMO=true npm run build --workspace @roport/pad -- --outDir .cache/dist-shopping-demo`로 별도 빌드하고, 로컬 쇼핑 시험 실행부에서 `apps/pad/.cache/dist-shopping-demo`를 제공합니다. 데모 예약의 서비스 선택 화면에 **외형 추종 단독 시험** 버튼이 따로 뜹니다. 일반 빌드에는 이 진입점이 없고, `VITE_PAD_API=server` 모드에서는 플래그가 있어도 열리지 않습니다.

e2e(`e2e/`, CI `e2e-ci.yml`): 서비스 서버를 위 조건(`SERVICE_DEV_LOGIN=true`, `SERVICE_DEV_CART=true`, 개발 시드 `db/dev/seed.sql` 적용)으로 띄운 뒤 `npx playwright install chromium`(처음 한 번), `npm run test:e2e`로 실행합니다. 패드(5185, 서버 모드)와 고객 웹(5186) 개발 서버는 Playwright가 띄우고, 예약은 고객 API로 만듭니다. 고객 웹 시나리오는 화면이 받은 예약 QR이 카트 확인에서 통과하는지와 만료 후 재발급을 봅니다.

각 앱의 `dist/`를 별도로 배포할 수 있습니다. 현재 기능은 준비 화면뿐입니다.
브라우저에 전달하는 환경변수에 비밀키를 넣지 않습니다.

### 쇼핑 모드

카트 패드에서 입국장 물품을 확인하고 **쇼핑 모드 · 짐 운반**을 누른다. **내 모습 찾기**로 한 장을 고정한 뒤 번호를 누르거나 **직접 선택**으로 몸통을 드래그한다. 검출 박스가 없으면 바로 드래그할 수 있다. 같은 대상과 센서가 확인된 뒤 **따라가기 시작**을 누른다. **잠시 멈추기** 후 같은 대상을 재개하거나 쇼핑을 종료할 수 있다.

이 화면은 Pinky의 로컬 `shopping_follow` 실행부가 제공하는 동일 출처의 `/shopping/` API를 사용한다. 단독 시험은 해당 실행부가 빌드된 패드를 함께 제공한다. 패드 정적 배포와 관제 접수만으로 실물 추종을 시작하지 않는다. [Pinky 쇼핑 실행 방법](../roport-pinky/src/roport_smartcart/README.md#쇼핑-추종)
