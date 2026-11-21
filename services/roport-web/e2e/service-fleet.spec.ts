// Explicit separate config: run against tools/run-latest-service-test.sh, never mock APIs.
import { readFileSync } from 'node:fs';
import { expect, test } from '@playwright/test';

test('customer browser → service WSS → MQTT robots → pad receipt → return (small and large)', async ({ browser, request }, testInfo) => {
  test.skip(!process.env.ROPORT_SERVICE_RUN, 'ROPORT_SERVICE_RUN must point to the local launcher run.json');
  const run = JSON.parse(readFileSync(process.env.ROPORT_SERVICE_RUN!, 'utf8'));
  const snapshot = async () => (await (await request.get(run.control_url + '/api/snapshot?mode=fleet')).json()).fleet;
  // Launching, opening the UI and entering its key must not dispatch robot work.
  await expect.poll(async () => Object.keys((await snapshot()).tasks).length).toBe(2);
  const operator = await browser.newPage();
  await operator.goto(run.control_url + '/ui/?mode=fleet');
  await expect(operator.locator('body')).toHaveAttribute('data-ready', 'true');
  await expect(operator.locator('#fleet-state')).toHaveText('요청 처리 시작 대기');
  await expect(operator.getByRole('button', { name: '요청 처리 시작', exact: true })).toBeDisabled();
  await operator.getByLabel('관제 조작 키').fill(readFileSync(run.key_file, 'utf8').trim());
  await expect(operator.getByRole('button', { name: '요청 처리 시작', exact: true })).toBeEnabled();
  // Observe multiple runtime ticks with the key entered, then reload without starting.
  for (let sample = 0; sample < 3; sample++) {
    const s = await snapshot();
    expect(s.enabled).toBe(false);
    expect(s.fault).toBeNull();
    expect(s.commands).toEqual({});
    expect(Object.values(s.tasks).map((t: any) => t.stage)).toEqual(['queued', 'queued']);
    await operator.waitForTimeout(500);
  }
  await operator.reload();
  await expect(operator.locator('#fleet-state')).toHaveText('요청 처리 시작 대기');
  await operator.getByLabel('관제 조작 키').fill(readFileSync(run.key_file, 'utf8').trim());
  await operator.getByRole('button', { name: '요청 처리 시작', exact: true }).click();
  await expect.poll(async () => (await snapshot()).enabled).toBe(true);
  await expect.poll(async () => Object.keys((await snapshot()).commands).length).toBeGreaterThan(0);
  await operator.close();
  await expect.poll(async () => {
    const s = await snapshot();
    expect(s.fault).toBeNull();
    return Object.values(s.tasks).filter((t: any) => t.status === 'COMPLETE').length;
  }, { timeout: 600_000, intervals: [1000] }).toBe(2);

  for (const login of run.accounts) {
    const context = await browser.newContext();
    const customer = await context.newPage();
    await customer.goto(run.customer_url);
    await customer.getByLabel('로그인 ID').fill(login);
    await customer.getByRole('button', { name: '로그인', exact: true }).click();
    await customer.getByRole('button', { name: '수령 예약하기' }).click();
    await customer.getByRole('radio', { name: '공용공간 시험 위치' }).check();
    await customer.getByRole('combobox').selectOption('e2e-flight');
    await customer.locator('input[name="pickupAt"]').first().check();
    const created = customer.waitForResponse(r => r.url().endsWith('/api/pickups') && r.request().method() === 'POST');
    await customer.getByRole('button', { name: /예약하기$/ }).click();
    const reservation = await (await created).json();
    expect(reservation.reservationNo).toHaveLength(8);
    await customer.getByRole('button', { name: '진행 상태 보기' }).click();
    await expect(customer.getByRole('heading', { name: /예약이 접수|카트에 물품|카트가 수령/ })).toBeVisible();
    // Customer polling cannot create another reservation or robot task.
    const duplicate = await context.request.post(run.customer_url + '/api/pickups', { data: { orderId: reservation.orderId, locationId: reservation.location.locationId, flightId: reservation.flight.flightId, pickupAt: reservation.pickupAt } });
    expect(duplicate.status()).toBe(409);

    const pad = await context.newPage();
    await pad.goto(run.pad_url);
    await pad.getByRole('button', { name: /예약번호 입력/ }).click();
    for (const ch of reservation.reservationNo) await pad.getByRole('button', { name: ch, exact: true }).click();
    const verified = pad.waitForResponse(r => r.url().endsWith('/cart-api/verifications'));
    await pad.getByRole('button', { name: '확인', exact: true }).click();
    const verification = await (await verified).json();
    await expect(pad.getByRole('heading', { name: '시험 고객님의 주문 물품' })).toBeVisible();
    await pad.getByRole('button', { name: '물품을 꺼냈습니다' }).click();
    // A valid reservation alone cannot complete an undelivered order.
    await pad.getByRole('button', { name: '수령 확인', exact: true }).click();
    await expect(pad.getByRole('alert')).toContainText('수령 확인이 저장되지 않았습니다');
    await expect(customer.getByRole('heading', { name: '카트가 도착했습니다. 패드에서 수령을 확인해 주세요' })).toBeVisible({ timeout: 300_000 });
    await customer.screenshot({ path: testInfo.outputPath(login + '-arrived.png'), fullPage: true });
    const receipt = pad.waitForResponse(r => r.url().endsWith('/cart-api/receipts'));
    await pad.getByRole('button', { name: '수령 확인', exact: true }).click();
    const response = await receipt;
    expect(response.ok()).toBeTruthy();
    const saved = await response.json();
    expect(saved.receiptId).toBeTruthy();
    await expect(pad.getByRole('heading', { name: '수령이 완료되었습니다' })).toBeVisible();
    await expect(customer.getByRole('heading', { name: '수령이 완료되었습니다' })).toBeVisible();
    const repeated = await request.post(run.service_url + '/cart-api/receipts', {
      headers: { 'X-Cart-Id': 'pinky3' },
      data: { verificationId: verification.verificationId, confirmationRequestId: 'receipt-' + verification.verificationId },
    });
    expect((await repeated.json()).receiptId).toBe(saved.receiptId);
    const wrongCart = await request.post(run.service_url + '/cart-api/receipts', {
      headers: { 'X-Cart-Id': 'pinky2' },
      data: { verificationId: verification.verificationId, confirmationRequestId: 'wrong-cart-' + verification.verificationId },
    });
    expect(wrongCart.status()).toBe(409);
    await expect.poll(async () => {
      const s = await snapshot();
      expect(s.fault).toBeNull();
      const task = Object.values(s.tasks).find((t: any) => t.request.operation === 'outbound' && t.request.package_id === saved.packageIds[0]) as any;
      return task?.status;
    }, { timeout: 300_000, intervals: [1000] }).toBe('COMPLETE');
    await pad.screenshot({ path: testInfo.outputPath(login + '-receipt.png'), fullPage: true });
    await context.close();
  }
});
