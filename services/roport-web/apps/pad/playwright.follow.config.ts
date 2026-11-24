import { defineConfig } from '@playwright/test';

// Isolated HTTP fixtures; no service database, MQTT broker, camera or robot is contacted.
export default defineConfig({
  testDir: './tests',
  testMatch: 'follow.spec.ts',
  outputDir: '/tmp/roport-pad-follow-results',
  workers: 1,
  use: { baseURL: 'http://127.0.0.1:5197', viewport: { width: 1280, height: 900 } },
  webServer: {
    command: 'npm run dev -- --port 5197 --strictPort',
    cwd: import.meta.dirname,
    env: { VITE_PAD_API: 'server', VITE_PAD_FOLLOW: '1' },
    url: 'http://127.0.0.1:5197',
    reuseExistingServer: false,
  },
});
