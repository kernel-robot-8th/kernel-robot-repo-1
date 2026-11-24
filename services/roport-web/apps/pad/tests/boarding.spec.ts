import { expect, test } from '@playwright/test';

const receipt = 'receipt-boarding';
const base = { receiptId: receipt, mode: 'marker', stage: 'follow_start', status: 'RUNNING', reason: null,
  updatedAt: new Date().toISOString(), revision: 1, stale: false, allowedActions: ['pause', 'return'], registration: null };
type Posts = Record<string, unknown>[];

async function open(page: import('@playwright/test').Page, current: () => Record<string, unknown>, cartPosts: Posts, robot: boolean[], failing = () => false) {
  await page.addInitScript(id => sessionStorage.setItem('roport.pad.receipt.pinky3', id), receipt);
  await page.route('**/cart-api/**', async route => {
    if (failing()) return route.abort();
    if (route.request().method() === 'POST') {
      const action = route.request().postDataJSON(); cartPosts.push(action);
      return route.fulfill({ status: 202, json: { requestId: action.requestId, accepted: true } });
    }
    return route.fulfill({ json: current() });
  });
  await page.route('**/follow-tool/api/boarding_alert', async route => {
    const body = route.request().postDataJSON(); robot.push(body.on);
    return route.fulfill({ json: { on: body.on } });
  });
  await page.goto('/');
}

test('boarding alert shows only when the server flags it during actual following, and closing sends no cart command', async ({ page }) => {
  let state: Record<string, unknown> = { ...base, boardingAlert: true };
  const cartPosts: Posts = []; const robot: boolean[] = [];
  await open(page, () => state, cartPosts, robot);

  // 서버가 알려도 접수·시작 확인 단계에서는 띄우지 않는다.
  await expect(page.getByRole('heading', { name: '출발을 확인하고 있습니다' })).toBeVisible();
  await page.waitForTimeout(2500);
  await expect(page.getByRole('alertdialog')).toHaveCount(0);

  // 실제 추종 중이어도 서버 알림이 없으면 띄우지 않는다(패드는 시간을 재지 않는다).
  state = { ...base, revision: 2, stage: 'following', allowedActions: ['pause'] };
  await expect(page.getByRole('heading', { name: '카트가 따라가고 있습니다' })).toBeVisible();
  await page.waitForTimeout(2500);
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  expect(robot).toEqual([]);

  // 추종 중 + 서버 알림이면 뜬다.
  state = { ...state, revision: 3, boardingAlert: true };
  const dialog = page.getByRole('alertdialog', { name: '탑승 시작 15분 전입니다' });
  await expect(dialog).toBeVisible();
  await expect(dialog.getByText('탑승 시각과 게이트를 확인해 주세요.', { exact: false })).toBeVisible();
  await expect(dialog.getByRole('button', { name: '확인' })).toBeFocused();
  await expect(dialog.getByRole('button', { name: /게이트 안내로 전환/ })).toHaveCount(0);
  await expect.poll(() => robot).toEqual([true]);

  // 닫으면 로봇 표시를 끄고, 카트 명령은 하나도 보내지 않았다.
  await page.keyboard.press('Escape');
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  await expect.poll(() => robot).toEqual([true, false]);
  expect(cartPosts).toEqual([]);

  // 고객이 닫은 뒤에는 서버 알림이 남아 있어도 같은 수령에서 새로고침해도 다시 띄우지 않는다.
  await page.reload();
  await expect(page.getByRole('heading', { name: '카트가 따라가고 있습니다' })).toBeVisible();
  await page.waitForTimeout(2500);
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  expect(robot).toEqual([true, false]);
  await expect(page.getByRole('button', { name: '잠시 멈추기' })).toBeEnabled();
});

test('pause inside the open boarding alert sends exactly one pause request', async ({ page }) => {
  let state: Record<string, unknown> = { ...base, revision: 2, stage: 'following', allowedActions: ['pause'], boardingAlert: true };
  const cartPosts: Posts = []; const robot: boolean[] = [];
  await open(page, () => state, cartPosts, robot);
  const dialog = page.getByRole('alertdialog');
  await expect(dialog).toBeVisible();
  const pause = dialog.getByRole('button', { name: '잠시 멈추기' });
  await expect(pause).toBeEnabled();
  await pause.dblclick();                       // 연속으로 눌러도 한 건
  await expect(pause).toBeDisabled();
  await expect.poll(() => cartPosts.length).toBe(1);
  expect(cartPosts[0]).toMatchObject({ operation: 'pause', expectedRevision: 2 });
  await page.waitForTimeout(2000);
  expect(cartPosts).toHaveLength(1);

  // 정지가 확인되면 창과 로봇 표시가 꺼지고 재개 버튼을 바로 쓸 수 있다.
  state = { ...state, revision: 3, stage: 'follow_paused', allowedActions: ['resume', 'return'] };
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  await expect.poll(() => robot).toEqual([true, false]);
  await expect(page.getByRole('button', { name: '추종 재개' })).toBeEnabled();
  expect(cartPosts).toHaveLength(1);
});

test('an open boarding alert and the robot display turn off when following is no longer confirmed', async ({ page }) => {
  let state: Record<string, unknown> = { ...base, revision: 2, stage: 'following', allowedActions: ['pause'], boardingAlert: true };
  let fail = false;
  const cartPosts: Posts = []; const robot: boolean[] = [];
  await open(page, () => state, cartPosts, robot, () => fail);
  await expect(page.getByRole('alertdialog')).toBeVisible();
  await expect.poll(() => robot).toEqual([true]);

  // HOLD 로 바뀌면 꺼진다.
  state = { ...state, revision: 3, status: 'HOLD' };
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  await expect.poll(() => robot).toEqual([true, false]);

  // 고객이 닫은 것이 아니므로 추종이 다시 확인되고 서버 알림이 남아 있으면 다시 뜬다.
  state = { ...state, revision: 4, status: 'RUNNING' };
  await expect(page.getByRole('alertdialog')).toBeVisible();
  await expect.poll(() => robot).toEqual([true, false, true]);

  // 상태 조회가 끊겨도 꺼진다.
  fail = true;
  await expect(page.getByText('최신 상태를 확인하지 못했습니다.', { exact: false })).toBeVisible({ timeout: 15000 });
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  await expect.poll(() => robot).toEqual([true, false, true, false]);

  // 서버가 오래된 상태라고 알릴 때도 띄우지 않는다.
  fail = false; state = { ...state, revision: 5, stale: true };
  await expect(page.getByRole('heading', { name: '카트가 따라가고 있습니다' })).toBeVisible();
  await page.waitForTimeout(2500);
  await expect(page.getByRole('alertdialog')).toHaveCount(0);
  expect(cartPosts).toEqual([]);
});
