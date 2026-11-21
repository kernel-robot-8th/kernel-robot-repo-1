import { defineConfig } from '@playwright/test';

export default defineConfig({
  testDir: './e2e',
  testMatch: 'service-fleet.spec.ts',
  timeout: 1800_000,
  workers: 1,
  use: { screenshot: 'only-on-failure', trace: 'retain-on-failure' },
});
