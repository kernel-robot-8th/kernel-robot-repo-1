import { defineConfig } from '@playwright/test';

export default defineConfig({
  testDir: './tests/browser',
  timeout: 45000,
  workers: 1,
  use: {
    baseURL: 'http://127.0.0.1:18765',
    viewport: { width: 1440, height: 1000 },
    launchOptions: { args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] },
  },
  webServer: {
    command: 'uv run uvicorn roport_control_server.adapters.api.app:app --host 127.0.0.1 --port 18765 --ws-max-size 4096 --ws-max-queue 4',
    url: 'http://127.0.0.1:18765/healthz',
    reuseExistingServer: false,
    timeout: 30000,
  },
});
