// 카트 패드 예약번호 확인 e2e: 패드 화면 → /cart-api 프록시 → 서비스 서버 → PostgreSQL.
// 예약은 고객 API(개발 로그인)로 만들고, 이미 진행 중인 예약이 있으면 그것을 쓴다(로컬 DB 재실행 대비).
import { expect, request, test, type Page } from '@playwright/test';
import { SERVICE_URL } from '../playwright.config';

type Item = { productName: string; quantity: number };
type Order = { orderId: string; businessStatus: string; items: Item[]; activePickup: { reservationNo: string } | null };

let reservationNo = '';
let items: Item[] = [];

test.beforeAll(async () => {
  const api = await request.newContext({ baseURL: SERVICE_URL });
  const health = await api.get('/actuator/health').catch(() => null);
  expect(health?.ok(), `서비스 서버가 ${SERVICE_URL}에서 떠 있어야 합니다(README의 e2e 실행 방법 참고)`).toBeTruthy();

  expect((await api.post('/api/session', { data: { loginId: 'demo1' } })).ok(), '개발 로그인(SERVICE_DEV_LOGIN=true)').toBeTruthy();
  const orders = (await (await api.get('/api/orders')).json()) as Order[];
  const order = orders.find((o) => o.businessStatus === 'CONFIRMED' && o.items.length > 0);
  expect(order, '개발 시드의 확정 주문').toBeTruthy();
  items = order!.items;

  if (order!.activePickup) {
    reservationNo = order!.activePickup.reservationNo;
  } else {
    const locationId = 'dev-loc-gate2';
    const flights = (await (await api.get('/api/flights')).json()) as Array<{ flightId: string }>;
    let created = false;
    for (const { flightId } of flights) {
      const q = new URLSearchParams({ flightId, locationId });
      const { times } = (await (await api.get(`/api/orders/${order!.orderId}/pickup-times?${q}`)).json()) as { times: string[] };
      if (!times?.length) continue;
      const res = await api.post('/api/pickups', { data: { orderId: order!.orderId, locationId, flightId, pickupAt: times[0] } });
      expect(res.status(), await res.text()).toBe(201);
      reservationNo = ((await res.json()) as { reservationNo: string }).reservationNo;
      created = true;
      break;
    }
    expect(created, '예약 가능한 항공편·시간').toBeTruthy();
  }
  await api.dispose();
});

async function enterReservationNo(page: Page, no: string) {
  await page.goto('/');
  await page.getByRole('button', { name: /예약번호 입력/ }).click();
  for (const ch of no) await page.getByRole('button', { name: ch, exact: true }).click();
  await page.getByRole('button', { name: '확인', exact: true }).click();
}

test('실제 예약번호로 확인하면 DB의 주문 물품을 보여 주고 수령 확인까지 간다', async ({ page }) => {
  const verifications: number[] = [];
  page.on('response', (r) => {
    if (r.url().endsWith('/cart-api/verifications')) verifications.push(r.status());
  });

  await enterReservationNo(page, reservationNo);
  await expect(page.getByRole('heading', { name: '테스트 고객 1님의 주문 물품' })).toBeVisible();
  for (const it of items) {
    await expect(page.getByRole('row', { name: new RegExp(it.productName) })).toContainText(`${it.quantity}개`);
  }
  await expect(page.getByText(`예약번호 ${reservationNo}`)).toBeVisible();
  expect(verifications).toEqual([201]);

  await page.getByRole('button', { name: '물품을 꺼냈습니다' }).click();
  await page.getByRole('button', { name: '수령 확인' }).click();
  await expect(page.getByRole('heading', { name: '수령이 완료되었습니다' })).toBeVisible();
  // 수령 기록 API가 없어 서버에는 저장되지 않는다는 안내가 보여야 한다.
  await expect(page.getByText('수령 기록은 아직 서버에 연결 전')).toBeVisible();
});

test('수령 기록이 저장되지 않으므로 같은 예약을 다시 확인할 수 있다(새 요청 ID)', async ({ page }) => {
  await enterReservationNo(page, reservationNo);
  await expect(page.getByRole('heading', { name: '테스트 고객 1님의 주문 물품' })).toBeVisible();
});

test('없는 예약번호는 다시 확인하라고 안내한다', async ({ page }) => {
  await enterReservationNo(page, 'ZZZZZZZZ');
  await expect(page.getByRole('alert')).toHaveText('예약을 찾을 수 없습니다. 예약번호를 다시 확인해 주세요.');
});
