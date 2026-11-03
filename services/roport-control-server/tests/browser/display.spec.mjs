import { test, expect } from '@playwright/test';

test('OFFLINE loads local 3D assets and retains detail tabs', async ({ page }) => {
  const errors = [];
  page.on('pageerror', (error) => errors.push(error.message));
  await page.goto('/ui/');
  await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
  await expect(page.locator('#mode')).toContainText('OFFLINE');
  await expect(page.locator('#source')).toContainText('실행부 미연결');
  await expect(page.locator('.robot')).toHaveCount(6);
  await expect(page.locator('canvas')).toHaveCount(2);
  await expect(page.locator('#tasks')).toContainText('수신된 작업이 없습니다');
  await expect(page.getByRole('button', { name: '작업 승인', exact: true })).toBeDisabled();
  await page.getByRole('button', { name: 'OMX 02 선택' }).click();
  await expect(page.locator('#name')).toHaveText('OMX 02');
  await page.getByRole('button', { name: 'OMX', exact: true }).click();
  await expect(page.locator('#omx')).toBeVisible();
  await page.getByRole('button', { name: 'Pinky', exact: true }).click();
  await expect(page.locator('#pinky')).toBeVisible();
  await page.getByRole('button', { name: '통합관제', exact: true }).click();
  await expect(page.locator('#name')).toHaveText('OMX 02');
  await page.getByRole('button', { name: '평면', exact: true }).click();
  await expect(page.locator('[data-view="top"]')).toHaveAttribute('aria-pressed', 'true');
  expect(errors).toEqual([]);
});

test('DEMO playback, stale state, disconnect and fresh reconnect', async ({ page, context }) => {
  const errors = [];
  page.on('pageerror', (error) => errors.push(error.message));
  await page.goto('/ui/?mode=demo');
  await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
  await expect(page.locator('#source')).toHaveText('모의 상태 수신');
  await expect(page.locator('#mode')).toContainText('DEMO');
  await expect(page.locator('.task')).toHaveCount(3);
  await page.locator('#play').click();
  await expect(page.locator('#play')).toHaveText('동작 재생');
  await page.locator('#drop').click();
  await expect(page.locator('.robot').first()).toHaveAttribute('data-stale', 'true', { timeout: 6000 });
  await page.locator('#drop').click();
  await expect(page.locator('.robot').first()).toHaveAttribute('data-stale', 'false');
  await context.setOffline(true);
  await expect(page.locator('#source')).toContainText('연결 끊김', { timeout: 10000 });
  await expect(page.locator('#play')).toBeDisabled();
  await context.setOffline(false);
  await expect(page.locator('#source')).toHaveText('모의 상태 수신', { timeout: 15000 });
  await expect(page.locator('#play')).toBeEnabled();
  await expect(page.locator('.robot').first()).toHaveAttribute('data-stale', 'false');
  expect(errors).toEqual([]);
});
