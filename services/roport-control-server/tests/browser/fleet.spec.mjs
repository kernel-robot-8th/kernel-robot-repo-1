import { test, expect } from '@playwright/test';
import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';

// This suite covers the real owner/socket/API/UI, not MQTT or simulated motion.
async function fixture(mode = 'simulation', receipt = false) {
  const child = spawn('.venv/bin/python', ['-u', 'tests/fleet_web_fixture.py', mode, ...(receipt ? ['--receipt'] : [])], {stdio: ['ignore', 'pipe', 'pipe']});
  const lines = createInterface({input: child.stdout});
  let diagnostics = '';
  child.stderr.on('data', data => { diagnostics += data; });
  const close = async () => {
    lines.close();
    child.kill('SIGINT');
    await new Promise(resolve => child.exitCode !== null ? resolve() : child.once('exit', resolve));
  };
  try {
    const ready = await new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error(`fleet fixture timeout: ${diagnostics}`)), 15000);
      lines.once('line', line => { clearTimeout(timer); resolve(JSON.parse(line)); });
      child.once('exit', code => { clearTimeout(timer); reject(new Error(`fleet fixture ${code}: ${diagnostics}`)); });
    });
    return {...ready, child, close};
  } catch (error) { await close(); throw error; }
}

test('fleet original UI: authenticate, change roles, accept requests, show stale status and reconnect', async ({page, context}, testInfo) => {
  const owner = await fixture();
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  try {
    await page.goto(`${owner.url}/ui/?mode=fleet`);
    await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
    await expect(page.locator('#fleet-panel')).toBeVisible();
    await expect(page.locator('#trial-panel')).toBeHidden();
    await expect(page.locator('.robot')).toHaveCount(6);
    await expect(page.locator('#fleet-shelf > div')).toHaveCount(6);
    await expect(page.locator('#fleet-state')).toHaveText('요청 처리 시작 대기');
    await expect(page.locator('#fleet-enable')).toBeDisabled();
    await expect(page.locator('#fleet-submit')).toBeDisabled();
    await page.locator('#fleet-key').fill('wrong');
    await page.locator('#fleet-enable').click();
    await expect(page.locator('#fleet-feedback')).toContainText('올바른 조작 키');
    await expect(page.locator('#fleet-state')).toHaveText('요청 처리 시작 대기');
    await page.locator('#fleet-key').fill(owner.token);
    await page.locator('#fleet-enable').click();
    await expect(page.locator('#fleet-state')).toHaveText('요청 처리 중');
    await page.locator('#fleet-robot').selectOption('pinky2');
    await page.locator('#fleet-role').selectOption('');
    await page.locator('#fleet-assign').click();
    await expect(page.locator('#fleet-roles')).toContainText('pinky2: 배정 제외');
    await page.locator('#fleet-role').selectOption('smartcart');
    await page.locator('#fleet-assign').click();
    await expect(page.locator('#fleet-roles')).toContainText('pinky2: 스마트카트');
    await page.locator('#fleet-operation').selectOption('outbound');
    await page.locator('#fleet-package').fill('missing-box');
    await page.locator('#fleet-submit').click();
    await expect(page.locator('#fleet-feedback')).toHaveText('출고할 물품이 선반에 등록되어 있지 않습니다.');
    await expect(page.locator('#fleet-work > li')).toHaveCount(0);
    await page.locator('#fleet-operation').selectOption('inbound');
    await page.locator('#fleet-package').fill('browser-box');
    await page.locator('#fleet-submit').click();
    await expect(page.locator('#fleet-work > li')).toHaveCount(1);
    await expect(page.locator('#fleet-work')).toContainText('browser-box · 배정 대기 · 배차 대기');
    await page.locator('#fleet-submit').click();
    await expect(page.locator('#fleet-feedback')).toContainText('active task');
    await expect(page.locator('#fleet-work > li')).toHaveCount(1);

    owner.child.kill('SIGUSR1');
    await expect(page.locator('[data-robot=pinky2]')).toHaveClass(/stale/, {timeout: 6000});
    await page.locator('#fleet-assign').click();
    await expect(page.locator('#fleet-feedback')).toContainText('stale');
    owner.child.kill('SIGUSR1');
    await expect(page.locator('[data-robot=pinky2]')).not.toHaveClass(/stale/);
    await context.setOffline(true);
    await expect(page.locator('#fleet-submit')).toBeDisabled({timeout: 5000});
    await context.setOffline(false);
    await expect(page.locator('#fleet-submit')).toBeEnabled({timeout: 8000});
    await expect(page.locator('#fleet-work > li')).toHaveCount(1);
    await page.reload();
    await expect(page.locator('#fleet-key')).toHaveValue('');
    await expect(page.locator('#fleet-submit')).toBeDisabled();
    await expect(page.locator('#fleet-work')).toContainText('browser-box');
    await page.screenshot({path: testInfo.outputPath('fleet-shared-state.png'), fullPage: true});
    expect(errors).toEqual([]);
  } finally { await context.setOffline(false); await owner.close(); }
});

test('fleet hardware view never permits a browser to fabricate customer receipt', async ({page}) => {
  const owner = await fixture('hardware', true);
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  try {
    await page.goto(`${owner.url}/ui/?mode=fleet`);
    await expect(page.locator('#fleet-work')).toContainText('고객 수령 대기');
    await page.locator('#fleet-key').fill(owner.token);
    await expect(page.getByRole('button', {name: '가상 고객 수령 확인'})).toHaveCount(0);
    const snapshot = await (await page.request.get(`${owner.url}/api/snapshot?mode=fleet`)).json();
    const response = await page.request.post(`${owner.url}/api/fleet/commands`, {
      headers: {'Origin': owner.url, 'X-Roport-Trial-Token': owner.token},
      data: {action: 'receipt', session_id: snapshot.fleet.session_id, request_id: 'await-receipt'},
    });
    expect(response.status()).toBe(409);
    expect((await response.json()).detail).toContain('service evidence');
    await expect(page.locator('#fleet-work')).toContainText('고객 수령 대기');
    expect(errors).toEqual([]);
  } finally { await owner.close(); }
});
