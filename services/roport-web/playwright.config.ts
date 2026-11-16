import { defineConfig } from '@playwright/test';

// 카트 패드 ↔ 실제 서비스 서버·PostgreSQL e2e.
// 서비스 서버(127.0.0.1:8080, SERVICE_DEV_LOGIN·SERVICE_DEV_CART=true, 개발 시드 적용)는 미리 띄워 둔다. 실행 방법은 README.
export const SERVICE_URL = process.env.E2E_SERVICE_URL ?? 'http://127.0.0.1:8080';

export default defineConfig({
  testDir: './e2e',
  timeout: 30000,
  workers: 1,
  use: {
    baseURL: 'http://127.0.0.1:5185',
    viewport: { width: 1280, height: 800 },
    screenshot: 'only-on-failure',
    trace: 'retain-on-failure',
  },
  webServer: {
    // 개발 서버의 /cart-api 프록시로 서비스 서버를 부른다(배포 nginx 전달은 아직 없음).
    command: 'npm run dev --workspace @roport/pad -- --port 5185 --strictPort',
    env: { VITE_PAD_API: 'server' },
    url: 'http://127.0.0.1:5185',
    reuseExistingServer: false,
    timeout: 30000,
  },
});
