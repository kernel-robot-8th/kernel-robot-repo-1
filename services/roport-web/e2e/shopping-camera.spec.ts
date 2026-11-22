import { expect, test } from '@playwright/test';
import { resolve } from 'node:path';
import { createServer } from 'vite';

test('느린 실시간 영상과 일시 실패한 선택 사진을 상태 폴링 중에도 표시한다', async ({ page }) => {
  // The shopping camera is only present in the standalone pad build.
  const server = await createServer({
    root: resolve('apps/pad'),
    define: { 'import.meta.env.VITE_PAD_API': 'undefined', 'import.meta.env.VITE_PAD_SHOPPING_DEMO': '"true"' },
    server: { host: '127.0.0.1', port: 0 },
  });
  try {
    await server.listen();
    let ownsCamera = false;
    let frame: string | null = null;
    let stillRequests = 0;
    let dropLive = false;
    let heartbeats = 0;
    const jpeg = await page.evaluate(() => document.createElement('canvas').toDataURL('image/jpeg').split(',')[1]);
    await page.route('**/shopping/**', async route => {
      const url = new URL(route.request().url());
      if (url.pathname === '/shopping/image') {
        if (dropLive && !url.searchParams.has('frame')) return route.fulfill({status: 404});
        if (url.searchParams.has('frame') && ++stillRequests === 1) {
          return route.fulfill({status: 404});
        }
        await new Promise(resolve => setTimeout(resolve, 550));
        return route.fulfill({contentType: 'image/jpeg', body: Buffer.from(jpeg, 'base64')});
      }
      if (url.pathname === '/shopping/preview') ownsCamera = true;
      if (url.pathname === '/shopping/heartbeat') heartbeats++;
      if (url.pathname === '/shopping/capture') frame = 'fixture';
      return route.fulfill({json: {ownsCamera, stopped: true, armed: false, selected: false,
        canStart: false, simulation: true, frame, detecting: false, boxes: [[0,0,.2,.4],[.6,0,.8,.4]]}});
    });
    await page.goto(server.resolvedUrls!.local[0]);
    const button = (name: string | RegExp) => page.getByRole('button', { name });
    await button('QR: 입국장 수령').click();
    await button('물품을 확인했습니다').click();
    await button(/외형 추종 단독 시험/).click();
    await button('카메라 켜기').click();
    await expect(page.getByText('실시간', {exact: true})).toBeVisible();
    expect(heartbeats).toBeGreaterThan(0);
    dropLive = true;
    await expect(page.getByText('카메라를 기다리는 중')).toBeVisible();
    dropLive = false;
    await expect(page.getByText('실시간', {exact: true})).toBeVisible();
    await button('내 모습 찾기').click();
    await expect(button('1번 선택')).toBeVisible();
    expect(stillRequests).toBe(2);
  } finally {
    await page.close();
    await server.close();
  }
});
