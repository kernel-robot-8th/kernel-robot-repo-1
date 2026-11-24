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

  // 고객이 닫은 뒤에는 같은 수령에서 새로고침해도 다시 띄우지 않는다.
  await page.reload();
  await expect(page.getByRole('heading', { name: '카트가 따라가고 있습니다' })).toBeVisible();
  await page.waitForTimeout(3500);
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  expect(robot).toEqual([true, false]);
  await expect(page.getByRole('button', { name: '잠시 멈추기' })).toBeEnabled();
});

test('an open boarding alert and the robot display turn off when following is no longer confirmed', async ({ page }) => {
  let state: Record<string, unknown> = { ...base, revision: 2, stage: 'following', allowedActions: ['pause'] };
  let fail = false;
  const robot: boolean[] = [];
  await page.addInitScript(id => sessionStorage.setItem('roport.pad.receipt.pinky3', id), receipt);
  await page.route('**/cart-api/**', async route => fail ? route.abort() : route.fulfill({ json: state }));
  await page.route('**/follow-tool/api/boarding_alert', async route => {
    const body = route.request().postDataJSON(); robot.push(body.on);
    return route.fulfill({ json: { on: body.on } });
  });
  await page.goto('/');
  await expect(page.getByRole('alertdialog')).toBeVisible({ timeout: 6000 });
  await expect.poll(() => robot).toEqual([true]);

  // 정지로 바뀌면 창과 로봇 표시가 꺼지고, 뒤의 재개 버튼을 바로 쓸 수 있다.
  state = { ...state, revision: 3, stage: 'follow_paused', allowedActions: ['resume', 'return'] };
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  await expect.poll(() => robot).toEqual([true, false]);
  await expect(page.getByRole('button', { name: '추종 재개' })).toBeEnabled();

  // 고객이 닫은 것이 아니므로 추종이 다시 확인되면 처음부터 세어 다시 알린다.
  state = { ...state, revision: 4, stage: 'following', allowedActions: ['pause'] };
  await expect(page.getByRole('alertdialog')).toBeVisible({ timeout: 6000 });
  await expect.poll(() => robot).toEqual([true, false, true]);

  // 상태 조회가 끊겨도 꺼진다.
  fail = true;
  await expect(page.getByText('최신 상태를 확인하지 못했습니다.', { exact: false })).toBeVisible({ timeout: 15000 });
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  await expect.poll(() => robot).toEqual([true, false, true, false]);

  // 서버가 오래된 상태라고 알릴 때도 세지 않는다.
  fail = false; state = { ...state, revision: 5, stale: true };
  await expect(page.getByRole('heading', { name: '카트가 따라가고 있습니다' })).toBeVisible();
  await page.waitForTimeout(3500);
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
});
