// 고객 웹 예약 QR e2e: 고객 웹 화면 → /api 프록시 → 서비스 서버 → PostgreSQL.
// 화면이 받은 QR 토큰이 카트 예약 확인에서 실제로 통과하는지, 만료 후 새 QR을 받으면 이전 QR이 폐기되는지 본다.
// 브라우저 시계만 앞당겨 만료를 재현한다(서버 시계는 그대로라 이전 QR의 거절 사유는 '폐기'다).
import { expect, request, test, type APIRequestContext } from '@playwright/test';
import { CUSTOMER_URL, SERVICE_URL } from '../playwright.config';

type Order = { orderId: string; businessStatus: string; items: unknown[]; activePickup: { reservationNo: string } | null };

let reservationNo = '';

test.beforeAll(async () => {
  const api = await request.newContext({ baseURL: SERVICE_URL });
  expect((await api.post('/api/session', { data: { loginId: 'demo1' } })).ok(), '개발 로그인(SERVICE_DEV_LOGIN=true)').toBeTruthy();
  const orders = (await (await api.get('/api/orders')).json()) as Order[];
  const order = orders.find((o) => o.businessStatus === 'CONFIRMED' && o.items.length > 0);
  expect(order, '개발 시드의 확정 주문').toBeTruthy();
  reservationNo = order!.activePickup?.reservationNo ?? (await reserve(api, order!.orderId));
  await api.dispose();
});

async function reserve(api: APIRequestContext, orderId: string): Promise<string> {
  const locationId = 'dev-loc-gate2';
  const flights = (await (await api.get('/api/flights')).json()) as Array<{ flightId: string }>;
  for (const { flightId } of flights) {
    const q = new URLSearchParams({ flightId, locationId });
    const { times } = (await (await api.get(`/api/orders/${orderId}/pickup-times?${q}`)).json()) as { times: string[] };
    if (!times?.length) continue;
    const res = await api.post('/api/pickups', { data: { orderId, locationId, flightId, pickupAt: times[0] } });
    expect(res.status(), await res.text()).toBe(201);
    return ((await res.json()) as { reservationNo: string }).reservationNo;
  }
  throw new Error('예약 가능한 항공편·시간이 없습니다');
}

// 카트 패드 대신 카트 API로 QR을 확인한다. 다른 시나리오의 카트를 잠그지 않게 카트 ID를 따로 쓴다.
async function verifyQr(token: string): Promise<{ status: number; result?: string; reasonCode?: string | null }> {
  const cart = await request.newContext({ baseURL: SERVICE_URL, extraHTTPHeaders: { 'X-Cart-Id': `e2e-qr-${Date.now()}` } });
  const res = await cart.post('/cart-api/verifications', {
    data: { verificationRequestId: `e2e-qr-${Date.now()}-${Math.random().toString(16).slice(2)}`, method: 'QR', value: token },
  });
  const body = (await res.json()) as { result?: string; reasonCode?: string | null };
  await cart.dispose();
  return { status: res.status(), ...body };
}

test('예약 QR을 그리고 그 QR로 카트 확인이 되며, 만료 후 새 QR을 받으면 이전 QR은 폐기된다', async ({ page }) => {
  await page.clock.install();
  await page.goto(CUSTOMER_URL);
  await page.getByLabel('로그인 ID').fill('demo1');
  await page.getByRole('button', { name: '로그인' }).click();
  await page.getByRole('button', { name: '내 예약', exact: true }).click();

  const first = page.waitForResponse((r) => r.url().endsWith('/qr-token') && r.request().method() === 'POST');
  await page.getByRole('listitem').filter({ hasText: reservationNo }).getByRole('button', { name: '예약 QR' }).click();
  const firstRes = await first;
  expect(firstRes.status()).toBe(201);
  const token1 = ((await firstRes.json()) as { token: string }).token;

  const qr = page.getByRole('img', { name: `예약 ${reservationNo} QR 코드` });
  await expect(qr).toBeVisible();
  expect(await qr.getAttribute('src')).toMatch(/^data:image\/png;base64,/); // 브라우저 안에서 그림(외부 요청 없음)
  await expect(page.getByText(/^\d+:\d{2} 후 만료/)).toBeVisible();
  expect(await verifyQr(token1)).toMatchObject({ status: 201, result: 'VERIFIED' });

  // 10분이 지나면 QR을 가리고 새 QR 받기를 보여 준다.
  await page.clock.fastForward('10:01');
  await expect(page.getByText('QR이 만료되었습니다')).toBeVisible();
  await expect(qr).toBeHidden();

  const second = page.waitForResponse((r) => r.url().endsWith('/qr-token') && r.request().method() === 'POST');
  await page.getByRole('button', { name: '새 QR 받기' }).click();
  const token2 = ((await (await second).json()) as { token: string }).token;
  expect(token2).not.toBe(token1);
  await expect(qr).toBeVisible();
  expect(await verifyQr(token2)).toMatchObject({ status: 201, result: 'VERIFIED' });
  expect(await verifyQr(token1)).toMatchObject({ status: 201, result: 'REJECTED', reasonCode: 'QR_REVOKED' });
});
