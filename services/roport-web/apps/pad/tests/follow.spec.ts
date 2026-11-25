import { expect, test } from '@playwright/test';

const receipt = 'receipt-test';
const base = { receiptId: receipt, mode: 'appearance', stage: 'follow_register', status: 'RUNNING', reason: null,
  updatedAt: new Date().toISOString(), revision: 1, stale: false, allowedActions: ['capture'], registration: null };
const registration = { frameRef: 'frame-test', imagePath: `/cart-api/follow/${receipt}/frames/frame-test`,
  expiresAt: new Date(Date.now() + 600_000).toISOString(), perceptionBootId: 'boot-test',
  candidates: [{ candidateId: 'person-test', box: [.2,.1,.7,.9] }] };

test('reload only reads status, explicit capture/selection waits for control and actual following', async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 });
  const fitsScreen = () => page.evaluate(() => document.documentElement.scrollWidth <= innerWidth);
  let state: Record<string, unknown> = { ...base };
  const actions: Record<string, unknown>[] = [];
  await page.addInitScript(id => sessionStorage.setItem('roport.pad.receipt.pinky3', id), receipt);
  await page.route('**/cart-api/**', async route => {
    const request = route.request();
    if (request.url().includes('/frames/')) return route.fulfill({ contentType: 'image/png', body: Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+a8ioAAAAASUVORK5CYII=', 'base64') });
    if (request.method() === 'POST') {
      const action = request.postDataJSON(); actions.push(action);
      if (action.operation === 'capture') state = { ...base, revision: 2, allowedActions: ['capture', 'select', 'return'], registration };
      // Acceptance does not confirm that the selected target or robot is ready.
      return route.fulfill({ status: 202, json: { requestId: action.requestId, accepted: true } });
    }
    return route.fulfill({ json: state });
  });
  await page.goto('/');
  await expect(page.getByRole('heading', { name: '함께 갈 사람을 등록해 주세요' })).toBeVisible();
  await page.reload();
  await expect(page.getByRole('button', { name: '내 모습 찾기' })).toBeEnabled();
  expect(actions).toHaveLength(0);
  await page.getByRole('button', { name: '내 모습 찾기' }).click();
  await expect(page.getByRole('button', { name: '1번 선택' })).toBeVisible();
  await expect(page.getByRole('img', { name: '대상 등록을 위해 촬영한 카트 앞 모습' })).toBeVisible();
  expect(await fitsScreen()).toBe(true);
  const camera = (await page.locator('.shopping-camera').boundingBox())!;
  const controls = (await page.locator('.shop-controls').boundingBox())!;
  expect(controls.y).toBeGreaterThanOrEqual(camera.y + camera.height);
  await page.getByRole('button', { name: '1번 선택' }).click();
  await expect.poll(() => actions.length).toBe(2);
  expect(actions[1]).toMatchObject({ expectedRevision: 2, operation: 'select', frameRef: 'frame-test', candidateId: 'person-test' });
  await expect(page.getByRole('heading', { name: '함께 갈 사람을 등록해 주세요' })).toBeVisible();
  await expect(page.getByRole('button', { name: '다시 찍기' })).toBeDisabled();
  state = { ...base, revision: 3, stage: 'follow_pending', allowedActions: ['return'], registration: { registered: true, expiresAt: registration.expiresAt, perceptionBootId: registration.perceptionBootId } };
  await expect(page.getByRole('heading', { name: '출발 승인을 기다리고 있습니다' })).toBeVisible();
  await page.reload();
  await expect(page.getByRole('heading', { name: '출발 승인을 기다리고 있습니다' })).toBeVisible();
  expect(actions).toHaveLength(2);
  state = { ...state, revision: 4, stage: 'following', allowedActions: ['pause'] };
  const pause = page.getByRole('button', { name: '잠시 멈추기' });
  await expect(pause).toBeEnabled();
  expect((await pause.boundingBox())!.height).toBeGreaterThanOrEqual(44);
  expect(await fitsScreen()).toBe(true);
  await pause.click();
  await expect.poll(() => actions.length).toBe(3);
  expect(actions[2].operation).toBe('pause');
  await expect(page.getByRole('heading', { name: '카트가 따라가고 있습니다' })).toBeVisible();
  state = { ...state, revision: 5, stage: 'follow_paused', allowedActions: ['resume', 'return'] };
  await expect(page.getByRole('heading', { name: '카트가 멈췄습니다' })).toBeVisible();
  await page.getByRole('button', { name: '추종 재개' }).click();
  await expect.poll(() => actions.length).toBe(4);
  expect(actions[3].operation).toBe('resume');
  state = { ...state, revision: 6, stale: true };
  await expect(page.getByText('최신 상태를 확인하지 못했습니다.', { exact: false })).toBeVisible();
  await expect(page.getByRole('button', { name: '물품을 모두 내렸습니다 · 복귀 요청' })).toBeDisabled();
  state = { ...state, revision: 7, stale: false, stage: 'follow_paused', allowedActions: ['return'] };
  await expect(page.getByRole('button', { name: '물품을 모두 내렸습니다 · 복귀 요청' })).toBeEnabled();
  await page.getByRole('button', { name: '물품을 모두 내렸습니다 · 복귀 요청' }).click();
  await expect.poll(() => actions.length).toBe(5);
  expect(actions[4]).toMatchObject({ operation: 'return', expectedRevision: 7 });
  state = { ...state, revision: 8, stage: 'return_nav', allowedActions: [] };
  await expect(page.getByRole('heading', { name: '카트가 복귀하고 있습니다' })).toBeVisible();
  state = { ...state, revision: 9, stage: 'done', status: 'COMPLETED' };
  await expect(page.getByRole('heading', { name: '이용이 완료되었습니다' })).toBeVisible();
  await page.reload();
  await expect(page.getByRole('heading', { name: '이용이 완료되었습니다' })).toBeVisible();
  expect(actions).toHaveLength(5);
});

test('lost receipt response restores fixed mode/card and never posts on reload', async ({ page }) => {
  const intent = { verificationId: 'verification-test', confirmationRequestId: 'receipt-verification-test', followMode: 'marker', followCard: 35 };
  const requests: unknown[] = [];
  await page.addInitScript(value => sessionStorage.setItem('roport.pad.receipt-pending.pinky3', JSON.stringify(value)), intent);
  await page.route('**/cart-api/**', async route => {
    if (route.request().method() === 'POST') {
      requests.push(route.request().postDataJSON());
      return route.fulfill({ status: 201, json: { receiptId: receipt, followSessionId: receipt } });
    }
    return route.fulfill({ json: { ...base, mode: 'marker', stage: 'follow_pending', allowedActions: ['return'] } });
  });
  await page.goto('/');
  await page.reload();
  await expect(page.getByRole('button', { name: '수령 확인 결과 다시 확인' })).toBeVisible();
  expect(requests).toHaveLength(0);
  await page.getByRole('button', { name: '수령 확인 결과 다시 확인' }).click();
  await expect(page.getByRole('heading', { name: '출발 승인을 기다리고 있습니다' })).toBeVisible();
  expect(requests).toEqual([intent]);
});

test('capability loading and unsupported appearance prevent selecting appearance', async ({ page }) => {
  await page.route('**/cart-api/verifications', route => route.fulfill({ status: 201, json: {
    verificationId: 'verification-test', result: 'VERIFIED', pickup: {
      pickupId: 'pickup-test', reservationNo: 'GT2X7KQA', orderNo: 'test-order', customerName: '가상 고객',
      location: { locationId: 'test-location', name: 'Gate 2' }, flight: { flightNo: 'ZZ202', gateCode: 'G2', boardingStart: '2026-11-24T10:00:00Z', boardingEnd: '2026-11-24T11:00:00Z' },
      items: [{ itemId: 'item-test', productName: '가상 상품', quantity: 1 }], packageCount: 1,
    },
  } }));
  const receipts: unknown[] = [];
  await page.route('**/cart-api/receipts', route => {
    receipts.push(route.request().postDataJSON());
    return route.fulfill({ status: 201, json: { receiptId: receipt, followSessionId: receipt } });
  });
  await page.route(`**/cart-api/follow/${receipt}`, route => route.fulfill({ json: { ...base, mode: 'marker', stage: 'follow_pending', allowedActions: ['return'] } }));
  let resolveCapabilities: () => void = () => {};
  const waitCapabilities = new Promise<void>(resolve => { resolveCapabilities = resolve; });
  await page.route('**/cart-api/follow/capabilities?*', async route => { await waitCapabilities; return route.fulfill({ json: { modes: ['marker'] } }); });
  await page.goto('/');
  await page.getByRole('button', { name: 'QR이 인식되지 않으면 예약번호 입력' }).click();
  await page.getByRole('textbox', { name: '예약번호' }).fill('GT2X7KQA');
  await page.getByRole('button', { name: '확인', exact: true }).click();
  await page.getByRole('button', { name: '물품을 꺼냈습니다' }).click();
  await expect(page.getByRole('radio', { name: '어려워요 · 외형 인식 추종' })).toBeDisabled();
  await expect(page.getByRole('radio', { name: '가능해요 · 마커 카드 추종' })).toBeDisabled();
  resolveCapabilities();
  await expect(page.getByRole('radio', { name: '가능해요 · 마커 카드 추종' })).toBeEnabled();
  await expect(page.getByRole('radio', { name: '어려워요 · 외형 인식 추종' })).toBeDisabled();
  await page.getByRole('radio', { name: '가능해요 · 마커 카드 추종' }).check();
  await page.getByRole('combobox', { name: '추종 카드 번호' }).selectOption('36');
  await page.getByRole('button', { name: '수령 확인 및 추종 요청' }).click();
  await expect(page.getByRole('heading', { name: '출발 승인을 기다리고 있습니다' })).toBeVisible();
  expect(receipts).toEqual([{ verificationId: 'verification-test', confirmationRequestId: 'receipt-verification-test', followMode: 'marker', followCard: 36 }]);
});
