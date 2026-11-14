import { test, expect } from '@playwright/test';
import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';

for (const scenario of ['single', 'two', 'smartcart']) {
for (const scope of ['line-only', 'nav2']) {
test(`${scenario}/${scope}: trial UI completes real MQTT simulation, reconnects and fences duplicate approvals`, async ({ page, context }, testInfo) => {
  const child = spawn('.venv/bin/python', ['-u', 'tests/trial_web_fixture.py', scope, scenario], { stdio: ['ignore', 'pipe', 'pipe'], env: {...process.env, ROPORT_TEST_OUTPUT_DIR: testInfo.outputPath('fixture')} });
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
    await page.addInitScript(() => {
      window.routeLineDraws = 0;
      for (const method of ['drawArrays', 'drawElements']) {
        const draw = WebGL2RenderingContext.prototype[method];
        WebGL2RenderingContext.prototype[method] = function (...args) {
          if (args[0] === this.LINE_STRIP) window.routeLineDraws++;
          return draw.apply(this, args);
        };
      }
    });
    // Reproduce opening the page before the owner supplies its routes.
    await page.route('**/api/bootstrap*', async (route) => {
      const response = await route.fetch();
      const data = await response.json();
      await route.fulfill({response, json: {...data, trial_routes: []}});
    });
    await page.goto(`${url}/ui/`);
    await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
    await expect(page.locator('#trial-phase')).toHaveText('출발 대기');
    await expect.poll(() => page.evaluate(() => window.routeLineDraws)).toBeGreaterThan(0);
    await expect(page.locator('.robot')).toHaveCount(6);
    await expect(page.locator('#map-name')).toContainText('실측 v14');
    await expect(page.locator('[data-robot=omx1]')).toContainText('설치 배치');
    await page.getByRole('button', {name: 'Pinky', exact: true}).click();
    await expect(page.locator('#trial-slots article')).toHaveCount(3);
    await expect(page.locator('#trial-slots [data-robot-id=pinky2]')).toContainText('시험 미배정');
    await expect(page.locator('#trial-pinky-panel #trial-panel')).toBeVisible();
    await expect(page.locator('[data-action="start"]')).toBeDisabled();
    await page.locator('#trial-key').fill(token);
    await expect(page.locator('#trial-scenario')).toContainText(scenario === 'single' ? '1대 시험' : scenario === 'two' ? '2대 시험' : '스마트카트 단독 시험');
    if (scenario === 'two') {
      await expect(page.locator('#trial-slots [data-robot-id=pinky3]')).toContainText('스마트카트');
      await expect(page.locator('#trial-request')).toBeDisabled();
    } else {
      await expect(page.locator('#trial-request')).toBeHidden();
    }
    await page.locator('[data-action="start"]').click();
    if (scenario === 'two') {
      await expect(page.locator('#trial-phase')).toHaveText('OMX 창고 상차 완료 승인 대기');
      await expect(page.locator('#trial-status')).toContainText('N01');
      await expect(page.locator('#trial-slots [data-robot-id=pinky3]')).toContainText('H1');
      await expect(page.locator('#trial-request')).toBeDisabled();
      await page.locator('#trial-dummy').click();
    }
    const firstWait = scenario === 'smartcart' ? 'OMX 카트 적재 완료 승인 대기' : 'OMX 입고 완료 승인 대기';
    await expect(page.locator('#trial-phase')).toHaveText(firstWait);
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
    await expect(page.locator('#trial-phase')).toHaveText(firstWait);
    // Keys are not persisted or put in the URL; another viewer sees the same task.
    await expect(page.locator('#trial-key')).toHaveValue('');
    const observer = await context.newPage();
    await observer.goto(`${url}/ui/`);
    await expect(observer.locator('#trial-phase')).toHaveText(firstWait);
    await page.locator('#trial-key').fill(token);
    if (scenario !== 'smartcart') await page.locator('#trial-dummy').click();
    if (scenario === 'two') {
      await expect(page.locator('#trial-phase')).toHaveText('물품 요청 대기');
      await expect(page.locator('#trial-slots [data-robot-id=pinky1]')).toContainText('N16');
      await expect(page.locator('#trial-slots [data-robot-id=pinky3]')).toContainText('H1');
      await page.reload();
      await expect(page.locator('#trial-phase')).toHaveText('물품 요청 대기');
      await page.locator('#trial-key').fill(token);
      await page.locator('#trial-request').click();
    }
    await expect(page.locator('#trial-phase')).toHaveText('OMX 카트 적재 완료 승인 대기');
    await expect(observer.locator('#trial-phase')).toHaveText('OMX 카트 적재 완료 승인 대기');
    if (scenario !== 'single') await expect(page.locator('#trial-status')).toContainText('pinky3');
    await page.locator('#trial-dummy').click();
    const complete = scope === 'line-only' ? 'N12 주행 완료 · Nav2 제외' : '시험 완료';
    await expect(page.locator('#trial-phase')).toHaveText(complete, {timeout: 15000});
    if (scenario !== 'single') await page.getByRole('button', {name: 'Pinky 03 선택', exact: true}).click();
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

for (const source of ['segment', 'delivery']) {
test(`composer/${source}: edit, validate, persist and execute through shared MQTT owner`, async ({page}, testInfo) => {
  const child = spawn('.venv/bin/python', ['-u', 'tests/trial_web_fixture.py', 'nav2', 'smartcart'], {stdio:['ignore','pipe','pipe'], env: {...process.env, ROPORT_TEST_OUTPUT_DIR: testInfo.outputPath('fixture')}});
  const lines = createInterface({input:child.stdout});
  let diagnostics = ''; child.stderr.on('data', d => {diagnostics += d;});
  try {
    const {url, token} = await new Promise((resolve,reject) => {
      const timer = setTimeout(() => reject(new Error(diagnostics || 'fixture timeout')), 20000);
      lines.once('line', s => {clearTimeout(timer); resolve(JSON.parse(s));});
      child.once('exit', code => {clearTimeout(timer); reject(new Error(`fixture ${code}: ${diagnostics}`));});
    });
    const errors = []; page.on('pageerror', e => errors.push(e.message));
    await page.goto(`${url}/ui/`);
    await page.locator('#trial-key').fill(token);
    await page.locator('#scenario-editor > summary').click();
    await expect(page.locator('#scenario-steps > li')).not.toHaveCount(0);
    if (source === 'segment') {
      while (await page.locator('#scenario-steps button').filter({hasText:'삭제'}).count()) {
        await page.locator('#scenario-steps button').filter({hasText:'삭제'}).first().click();
      }
      await page.locator('#scenario-template').selectOption({label:'라인 이동 · cart_to_delivery'});
      await page.locator('#scenario-add').click();
      await expect(page.locator('#scenario-steps > li')).toHaveCount(1);
      await page.locator('#scenario-name').fill('H1에서 N11 구간 검증');
      const nodes = page.locator('#scenario-steps input').nth(1);
      await nodes.fill('H1 N99 N11');
      await page.locator('#scenario-validate').click();
      await expect(page.locator('#scenario-feedback')).toContainText('disconnected/unknown');
      await nodes.fill('H1 N11');
      await page.getByRole('button', {name:'첫·끝 노드로 경로 계산'}).click();
      await expect(page.locator('#scenario-feedback')).toContainText('최소 간선 경로');
      await expect(page.locator('#scenario-steps input').nth(1)).toHaveValue('H1 N09 N24 N19 N11');
      await page.locator('#scenario-validate').click();
      await expect(page.locator('#scenario-feedback')).toContainText('설정 검증 통과');
      await page.locator('#scenario-save').click();
      await expect(page.locator('#scenario-feedback')).toContainText('서버 scenario-plans.json');
      await expect(page.locator('#scenario-saved')).toContainText('H1에서 N11');
      await page.locator('#scenario-load').click();
      await expect(page.locator('#scenario-feedback')).toContainText('불러왔습니다');
    } else {
      await page.locator('#scenario-fields details > summary').click();
      await page.locator('#delivery-nav2').check();
      await page.locator('#delivery-generate').click();
      await expect(page.locator('#scenario-feedback')).toContainText('요청 계획 생성');
      await expect(page.locator('#scenario-steps > li')).toHaveCount(5);
    }
    await page.locator('#scenario-apply').click();
    await expect(page.locator('#scenario-feedback')).toContainText('계획 적용 완료');
    await expect(page.locator('#trial-scenario')).toContainText(source === 'segment' ? 'H1에서 N11' : '출고 요청');
    await page.reload();
    await page.locator('#trial-key').fill(token);
    await page.locator('#scenario-editor > summary').click();
    await expect(page.locator('#scenario-steps > li')).toHaveCount(source === 'segment' ? 1 : 5);
    await page.screenshot({path:testInfo.outputPath('composer-ready.png'), fullPage:true});
    await page.locator('[data-action=start]').click();
    if (source === 'delivery') {
      await expect(page.locator('#trial-phase')).toHaveText('적재 완료 수동 확인');
      await expect(page.locator('#scenario-apply')).toBeDisabled();
      await page.locator('#trial-dummy').click();
    }
    await expect(page.locator('#trial-phase')).toHaveText('시험 완료', {timeout:15000});
    await expect(page.locator('#scenario-progress')).not.toContainText('대기');
    await page.screenshot({path:testInfo.outputPath('composer-complete.png'), fullPage:true});
    expect(errors).toEqual([]);
  } finally {
    lines.close(); child.kill('SIGINT');
    await new Promise(resolve => child.exitCode !== null ? resolve() : child.once('exit',resolve));
  }
});
}

for (const selected of ['single', 'two', 'smartcart']) {
test(`common owner/${selected}: select preset and edit nodes without scenario CLI flags`, async ({page}, testInfo) => {
  const participant = selected === 'single' ? 'logistics' : selected === 'smartcart' ? 'smartcart' : 'all';
  const child = spawn('.venv/bin/python', ['-u', 'tests/trial_web_fixture.py', 'nav2', 'two', participant], {stdio:['ignore','pipe','pipe'], env: {...process.env, ROPORT_TEST_OUTPUT_DIR: testInfo.outputPath('fixture')}});
  const lines = createInterface({input:child.stdout});
  let diagnostics = ''; child.stderr.on('data', d => {diagnostics += d;});
  try {
    const {url, token} = await new Promise((resolve,reject) => {
      const timer = setTimeout(() => reject(new Error(diagnostics || 'fixture timeout')), 20000);
      lines.once('line', s => {clearTimeout(timer); resolve(JSON.parse(s));});
      child.once('exit', code => {clearTimeout(timer); reject(new Error(`fixture ${code}: ${diagnostics}`));});
    });
    const errors = []; page.on('pageerror', e => errors.push(e.message));
    await page.goto(`${url}/ui/`);
    await page.locator('#trial-key').fill(token);
    await page.locator('#scenario-editor > summary').click();
    await expect(page.locator('#scenario-preset option')).toHaveCount(3);
    // Switching drafts does not start or apply a plan.
    for (const name of ['two', 'smartcart', 'single', selected]) {
      await page.locator('#scenario-preset').selectOption(name);
      await page.locator('#scenario-preset-load').click();
      await expect(page.locator('#trial-phase')).toHaveText('출발 대기');
    }
    if (selected === 'two') {
      await page.locator('#scenario-nav2').uncheck();
      await page.locator('#scenario-preset-load').click();
    }
    const routeInput = page.locator('#scenario-steps > li').first().getByLabel('노드 목록', {exact:true});
    const original = await routeInput.inputValue();
    await routeInput.fill('N99 N11');
    await page.locator('#scenario-validate').click();
    await expect(page.locator('#scenario-feedback')).toContainText('disconnected/unknown');
    const edited = selected === 'single' ? 'N16 N14 N15 N05 N07 N23 N19 N11' : original;
    await routeInput.fill(edited);
    await expect(page.locator('[data-action=start]')).toBeDisabled();
    await page.locator('#scenario-validate').click();
    await expect(page.locator('#scenario-feedback')).toContainText('설정 검증 통과');
    await page.locator('#scenario-apply').click();
    await expect(page.locator('#scenario-feedback')).toContainText('계획 적용 완료');
    const snapshot = await (await page.request.get(`${url}/api/snapshot?mode=trial`)).json();
    expect(snapshot.trial.plan.steps[0].payload.route).toEqual(edited.split(' '));
    await expect(page.locator('[data-action=start]')).toBeEnabled();
    await page.screenshot({path:testInfo.outputPath('preset-ready.png'), fullPage:true});
    await page.locator('[data-action=start]').click();
    await expect(page.locator('#scenario-preset-load')).toBeDisabled();
    if (selected === 'two') {
      await expect(page.locator('#trial-phase')).toHaveText('WAIT_LOAD');
      await page.locator('#trial-dummy').click();
    }
    if (selected !== 'smartcart') {
      await expect(page.locator('#trial-phase')).toHaveText('WAIT_INBOUND');
      await page.locator('#trial-dummy').click();
    }
    if (selected === 'two') {
      await expect(page.locator('#trial-phase')).toHaveText('WAIT_REQUEST');
      await page.locator('#trial-request').click();
    }
    await expect(page.locator('#trial-phase')).toHaveText('WAIT_CART_LOAD');
    await page.locator('#trial-dummy').click();
    await expect(page.locator('#trial-phase')).toHaveText('시험 완료', {timeout:15000});
    // Same owner remains available for the next selection, no process restart.
    await expect(page.locator('#scenario-preset-load')).toBeEnabled();
    await page.locator('#scenario-preset').selectOption(selected === 'single' ? 'smartcart' : 'single');
    await page.locator('#scenario-preset-load').click();
    await page.locator('#scenario-apply').click();
    await expect(page.locator('#trial-phase')).toHaveText('출발 대기');
    expect(errors).toEqual([]);
  } finally {
    lines.close(); child.kill('SIGINT');
    await new Promise(resolve => child.exitCode !== null ? resolve() : child.once('exit',resolve));
  }
});
}

test('hardware UI blocks key entry over remote HTTP', async ({ page, baseURL }) => {
  const errors = []; page.on('pageerror', e => errors.push(e.message));
  await page.route('http://control.example/**', async (route) => {
    const url = new URL(route.request().url());
    url.host = new URL(baseURL).host;
    await route.fulfill({ response: await route.fetch({ url: url.href }) });
  });
  await page.goto('http://control.example/ui/?mode=offline');
  await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
  const id = await page.evaluate(async () => (await import('/ui/scenario.js')).newRequestId());
  expect(id).toMatch(/^[a-f0-9]{32}$/);
  await page.evaluate(async () => {
    const { renderTrial } = await import('/ui/trial.js');
    document.getElementById('trial-key').value = 'test-only-operation-key';
    renderTrial({ robots: [], trial: {mode: 'hardware', available: true, can_start: true, phase: 'READY'} });
  });
  await expect(page.locator('#trial-key')).toBeDisabled();
  await expect(page.locator('#trial-key')).toHaveValue('');
  await expect(page.locator('#trial-readiness')).toContainText('HTTPS');
  await expect(page.locator('[data-action="start"]')).toBeDisabled();
  await expect(page.locator('#scenario-fields')).toHaveJSProperty('disabled', true);
  await expect(page.locator('#scenario-fields input').first()).toBeDisabled();
  await expect(page.locator('#service-fields')).toHaveJSProperty('disabled', true);
  expect(errors).toEqual([]);
});
