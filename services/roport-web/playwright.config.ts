import { defineConfig } from '@playwright/test';

// 카트 패드·고객 웹 ↔ 실제 서비스 서버·PostgreSQL e2e.
// 서비스 서버(127.0.0.1:8080, SERVICE_DEV_LOGIN·SERVICE_DEV_CART=true, 개발 시드 적용)는 미리 띄워 둔다. 실행 방법은 README.
export const SERVICE_URL = process.env.E2E_SERVICE_URL ?? 'http://127.0.0.1:8080';
// 고객 웹 개발 서버. /api를 서비스 서버로 넘긴다(apps/service/vite.config.ts).
export const CUSTOMER_URL = 'http://127.0.0.1:5186';

export default defineConfig({
  testDir: './e2e',
  testIgnore: 'service-fleet.spec.ts',
  timeout: 30000,
  workers: 1,
  use: {
    baseURL: 'http://127.0.0.1:5185',
    viewport: { width: 1280, height: 800 },
    screenshot: 'only-on-failure',
    trace: 'retain-on-failure',
  },
  webServer: [
    {
      // 개발 서버의 /cart-api 프록시로 서비스 서버를 부른다(배포 nginx 전달은 아직 없음).
      command: 'npm run dev --workspace @roport/pad -- --port 5185 --strictPort',
      env: { VITE_PAD_API: 'server', ROPORT_SERVICE_HTTP_URL: SERVICE_URL },
      url: 'http://127.0.0.1:5185',
      reuseExistingServer: false,
      timeout: 30000,
    },
    {
      command: 'npm run dev --workspace @roport/service -- --port 5186 --strictPort',
      url: CUSTOMER_URL,
      env: { ROPORT_SERVICE_HTTP_URL: SERVICE_URL },
      reuseExistingServer: false,
      timeout: 30000,
    },
  ],
});
