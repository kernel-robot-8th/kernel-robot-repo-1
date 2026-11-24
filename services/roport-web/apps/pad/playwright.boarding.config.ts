import { defineConfig } from '@playwright/test';

// 탑승 시간 알림(패드 타이머)만 본다. 서비스 DB·MQTT·카메라·로봇·인식 도구에는 닿지 않는다(HTTP 를 가로챈다).
export default defineConfig({
  testDir: './tests',
  testMatch: 'boarding.spec.ts',
  outputDir: '/tmp/roport-pad-boarding-results',
  workers: 1,
  use: { baseURL: 'http://127.0.0.1:5198', viewport: { width: 1280, height: 900 } },
  webServer: {
    command: 'npm run dev -- --port 5198 --strictPort',
    cwd: import.meta.dirname,
    env: { VITE_PAD_API: 'server', VITE_PAD_FOLLOW: '1', VITE_PAD_BOARDING_ALERT_S: '2', VITE_PAD_BOARDING_ROBOT: '1' },
    url: 'http://127.0.0.1:5198',
    reuseExistingServer: false,
  },
});
