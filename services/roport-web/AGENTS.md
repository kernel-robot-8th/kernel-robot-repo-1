# Web workspace

- Node.js 24 / npm workspaces / React / TypeScript / Vite를 사용한다. 서비스 웹은 `apps/service/`, 어드민은 `apps/admin/`이다.
- 의존성 설치와 lockfile 관리는 이 디렉토리에서 `npm ci`로 한다. 앱별 lockfile을 새로 만들지 않는다.
- 검사: `npm run typecheck`, `npm run build`. 앱 하나만 검사할 때는 해당 workspace를 지정한다.
- 공통 UI·타입이 실제로 반복될 때 기존 구조 안에서 공유한다. 두 앱을 미리 합치거나 별도 공유 패키지를 만들지 않는다.
- 로딩·실패·빈 결과·권한 상태를 구분하고 입력·접근성·키보드 사용을 확인한다. 프런트엔드에 비밀값을 넣지 않는다.
