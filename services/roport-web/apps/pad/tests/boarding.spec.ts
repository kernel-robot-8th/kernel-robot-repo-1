import { expect, test } from '@playwright/test';

const receipt = 'receipt-boarding';
const base = { receiptId: receipt, mode: 'marker', stage: 'follow_start', status: 'RUNNING', reason: null,
  updatedAt: new Date().toISOString(), revision: 1, stale: false, allowedActions: ['pause', 'return'], registration: null };

test('boarding alert appears once after actual following, drives the robot display, and never sends a cart command', async ({ page }) => {
  let state: Record<string, unknown> = { ...base };
  const cartPosts: unknown[] = [];
  const robot: boolean[] = [];
  await page.addInitScript(id => sessionStorage.setItem('roport.pad.receipt.pinky3', id), receipt);
  await page.route('**/cart-api/**', async route => {
    if (route.request().method() === 'POST') { cartPosts.push(route.request().postDataJSON()); return route.fulfill({ status: 202, json: { accepted: true } }); }
    return route.fulfill({ json: state });
  });
  await page.route('**/follow-tool/api/boarding_alert', async route => {
    const body = route.request().postDataJSON(); robot.push(body.on);
    return route.fulfill({ json: { on: body.on } });
  });
  await page.goto('/');

  // 접수·시작 확인 단계에서는 시간을 세지 않는다.
  await expect(page.getByRole('heading', { name: '출발을 확인하고 있습니다' })).toBeVisible();
  await page.waitForTimeout(3000);
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  expect(robot).toEqual([]);

  // 실제 추종이 확인된 뒤 설정한 시간(시험 2초)이 지나면 뜬다.
  state = { ...state, revision: 2, stage: 'following', allowedActions: ['pause'] };
  await expect(page.getByRole('heading', { name: '카트가 따라가고 있습니다' })).toBeVisible();
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  const dialog = page.getByRole('alertdialog', { name: '탑승 시작 15분 전입니다' });
  await expect(dialog).toBeVisible({ timeout: 6000 });
  await expect(dialog.getByText('탑승 시각과 게이트를 확인해 주세요.', { exact: false })).toBeVisible();
  await expect(dialog.getByRole('button', { name: '확인' })).toBeFocused();
  await expect(dialog.getByRole('button', { name: /게이트 안내로 전환/ })).toHaveCount(0);
  await expect.poll(() => robot).toEqual([true]);

  // 닫으면 로봇 표시를 끄고, 카트 명령은 하나도 보내지 않았다.
  await page.keyboard.press('Escape');
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  await expect.poll(() => robot).toEqual([true, false]);
  expect(cartPosts).toEqual([]);

  // 같은 수령에서는 새로고침·재개 뒤에도 다시 띄우지 않는다.
  await page.reload();
  await expect(page.getByRole('heading', { name: '카트가 따라가고 있습니다' })).toBeVisible();
  await page.waitForTimeout(3500);
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  expect(robot).toEqual([true, false]);
  await expect(page.getByRole('button', { name: '잠시 멈추기' })).toBeEnabled();
});
