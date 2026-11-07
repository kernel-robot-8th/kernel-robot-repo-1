import { test, expect } from '@playwright/test';
import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';

for (const scenario of ['single', 'two']) {
for (const scope of ['line-only', 'nav2']) {
test(`${scenario}/${scope}: trial UI completes real MQTT simulation, reconnects and fences duplicate approvals`, async ({ page, context }, testInfo) => {
  const child = spawn('.venv/bin/python', ['-u', 'tests/trial_web_fixture.py', scope, scenario], { stdio: ['ignore', 'pipe', 'pipe'] });
  let diagnostics = '';
  child.stderr.on('data', (data) => { diagnostics += data; });
  const lines = createInterface({ input: child.stdout });
  try {
    const { url, token } = await new Promise((resolve, reject) => {
      const timeout = setTimeout(() => reject(new Error(`fixture timeout: ${diagnostics}`)), 20000);
      child.once('exit', (code) => { clearTimeout(timeout); reject(new Error(`fixture exited ${code}: ${diagnostics}`)); });
      lines.once('line', (line) => { clearTimeout(timeout); resolve(JSON.parse(line)); });
    });
    const errors = [];
    page.on('pageerror', (error) => errors.push(error.message));
    await page.goto(`${url}/ui/`);
    await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
    await expect(page.locator('#trial-phase')).toHaveText('출발 대기');
    await expect(page.locator('.robot')).toHaveCount(6);
    await expect(page.locator('#map-name')).toContainText('실측 v14');
    await expect(page.locator('[data-robot=omx1]')).toContainText('설치 배치');
    await page.getByRole('button', {name: 'Pinky', exact: true}).click();
    await expect(page.locator('#trial-slots article')).toHaveCount(3);
    await expect(page.locator('#trial-slots [data-robot-id=pinky2]')).toContainText('시험 미배정');
    await expect(page.locator('#trial-pinky-panel #trial-panel')).toBeVisible();
    await expect(page.locator('[data-action="start"]')).toBeDisabled();
    await page.locator('#trial-key').fill(token);
    await page.locator('[data-action="start"]').click();
    if (scenario === 'two') {
      await expect(page.locator('#trial-phase')).toHaveText('OMX 창고 상차 완료 승인 대기');
      await page.locator('#trial-dummy').click();
    }
    await expect(page.locator('#trial-phase')).toHaveText('OMX 입고 완료 승인 대기');
    await expect(page.locator('#trial-status')).toContainText('N11');
    await page.getByRole('button', {name: '통합관제', exact: true}).click();
    await expect(page.locator('#trial-overview #trial-panel')).toBeVisible();
    const previewColors = await page.locator('#model canvas').evaluate((canvas) => {
      const copy = document.createElement('canvas');
      copy.width = canvas.width; copy.height = canvas.height;
      const ctx = copy.getContext('2d');
      ctx.drawImage(canvas, 0, 0);
      const pixels = ctx.getImageData(0, 0, copy.width, copy.height).data;
      const colors = new Set();
      for (let i = 0; i < pixels.length; i += 4 * 37) colors.add(`${pixels[i]},${pixels[i + 1]},${pixels[i + 2]},${pixels[i + 3]}`);
      return colors.size;
    });
    expect(previewColors).toBeGreaterThan(20); // Stationary model must not clear to a blank canvas.
    await page.screenshot({path: testInfo.outputPath('trial-inbound.png'), fullPage: true});
    await page.reload();
    await expect(page.locator('#trial-phase')).toHaveText('OMX 입고 완료 승인 대기');
    // Keys are not persisted or put in the URL; another viewer sees the same task.
    await expect(page.locator('#trial-key')).toHaveValue('');
    const observer = await context.newPage();
    await observer.goto(`${url}/ui/`);
    await expect(observer.locator('#trial-phase')).toHaveText('OMX 입고 완료 승인 대기');
    await page.locator('#trial-key').fill(token);
    await page.locator('#trial-dummy').click();
    if (scenario === 'two') {
      await expect(page.locator('#trial-phase')).toHaveText('시험 물품 요청 대기');
      await expect(page.locator('#trial-package')).toContainText('trial-package-1');
      await page.locator('#trial-request').click();
    }
    await expect(page.locator('#trial-phase')).toHaveText('OMX 카트 적재 완료 승인 대기');
    await expect(observer.locator('#trial-phase')).toHaveText('OMX 카트 적재 완료 승인 대기');
    await page.locator('#trial-dummy').click();
    const complete = scope === 'line-only' ? 'N12 주행 완료 · Nav2 제외' : '시험 완료';
    await expect(page.locator('#trial-phase')).toHaveText(complete, {timeout: 15000});
    if (scenario === 'two') await page.getByRole('button', {name: 'Pinky 03 선택', exact: true}).click();
    await expect(page.locator('#telemetry')).toContainText(scope === 'line-only' ? '마지막 확인 노드' : 'Nav2 완료 응답 위치');
    await expect(page.locator('[data-action="start"]')).toBeDisabled();
    await page.screenshot({path: testInfo.outputPath('trial-complete.png'), fullPage: true});
    await context.setOffline(true);
    await expect(page.locator('#source')).toContainText('연결 끊김', {timeout: 10000});
    await expect(page.locator('#trial-dummy')).toBeDisabled();
    await context.setOffline(false);
    await expect(page.locator('#trial-phase')).toHaveText(complete);
    expect(errors).toEqual([]);
  } finally {
    lines.close();
    child.kill('SIGINT');
    await new Promise((resolve) => child.exitCode !== null ? resolve() : child.once('exit', resolve));
  }
});
}
}
