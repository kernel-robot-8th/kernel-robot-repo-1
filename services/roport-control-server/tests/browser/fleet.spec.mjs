import { test, expect } from '@playwright/test';
import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';

// This suite covers the real owner/socket/API/UI, not MQTT or simulated motion.
async function fixture(mode = 'simulation', receipt = false, held = false, extra = []) {
  const child = spawn('.venv/bin/python', ['-u', 'tests/fleet_web_fixture.py', mode, ...extra, ...(receipt ? ['--receipt'] : []), ...(held ? ['--held'] : []), ...(held === 'rejected' ? ['--rejected'] : [])], {stdio: ['ignore', 'pipe', 'pipe']});
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
  await page.addInitScript(() => { Object.defineProperty(crypto, 'randomUUID', {value: undefined}); });
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


test('held fleet shows owner evidence without changing commands, stock or reservations', async ({page, context}, testInfo) => {
  const owner = await fixture('simulation', false, true);
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  try {
    const snapshot = async () => (await (await page.request.get(`${owner.url}/api/snapshot?mode=fleet`)).json()).fleet;
    const before = await snapshot();
    await page.goto(`${owner.url}/ui/?mode=fleet`);
    await expect(page.locator('#fleet-diagnostics')).toHaveAttribute('open', '');
    await expect(page.locator('#fleet-diagnostic-status')).toContainText('owner_restarted');
    await expect(page.locator('#fleet-pending-commands')).toContainText('held-command');
    await expect(page.locator('#fleet-pending-commands')).toContainText('held-box');
    await expect(page.locator('#fleet-reservation-owners')).toContainText('held-box');
    await expect(page.locator('#fleet-robot-observations')).toContainText('N16');
    await page.locator('#fleet-key').fill(owner.token);
    await expect(page.locator('#fleet-enable')).toBeDisabled();
    owner.child.kill('SIGUSR1');
    await expect(page.locator('#fleet-robot-observations')).toContainText('오래된 관측', {timeout: 6000});
    await context.setOffline(true);
    await expect(page.locator('#fleet-diagnostic-status')).toContainText('마지막 수신 상태', {timeout: 5000});
    await context.setOffline(false);
    await page.reload();
    await expect(page.locator('#fleet-pending-commands')).toContainText('held-command');
    const after = await snapshot();
    for (const key of ['commands', 'tasks', 'slots', 'resources', 'fault', 'enabled']) expect(after[key]).toEqual(before[key]);
    await page.locator('#fleet-diagnostics').screenshot({path: testInfo.outputPath('fleet-held-evidence.png')});
    expect(errors).toEqual([]);
  } finally { await context.setOffline(false); await owner.close(); }
});


test('operator reconciles a rejected command, releases HOLD and separately enables dispatch', async ({page}) => {
  const owner = await fixture('simulation', false, 'rejected');
  try {
    await page.goto(`${owner.url}/ui/?mode=fleet`);
    await expect(page.locator('#fleet-recovery-note')).toBeDisabled();
    await page.locator('#fleet-key').fill(owner.token);
    await expect(page.locator('#fleet-recovery-note')).toBeEnabled();
    await expect(page.locator('#fleet-reconcile')).toBeDisabled();
    await page.locator('#fleet-recovery-note').fill('현장 기체 정지 및 미착수 확인');
    await page.locator('#fleet-recovery-confirm').check();
    await page.locator('#fleet-reconcile').click();
    await expect(page.locator('#fleet-feedback')).toContainText('matching completion evidence');
    await expect(page.locator('#fleet-release')).toBeDisabled();
    await page.locator('#fleet-recovery-outcome').selectOption('not_started');
    await page.locator('#fleet-recovery-confirm').check();
    await page.locator('#fleet-reconcile').click();
    await expect(page.locator('#fleet-recovery-command option')).toHaveCount(0);
    await expect(page.locator('#fleet-enable')).toBeDisabled();
    await page.locator('#fleet-recovery-confirm').check();
    await page.locator('#fleet-release').click();
    await expect(page.locator('#fleet-state')).toHaveText('요청 처리 시작 대기');
    await expect(page.locator('#fleet-recovery-note')).toBeDisabled();
    await page.locator('#fleet-enable').click();
    await expect(page.locator('#fleet-state')).toHaveText('요청 처리 중');
  } finally { await owner.close(); }
});


test('restarted arrival requires an explicit confirmation and a separate restart', async ({page}) => {
  const owner = await fixture('simulation', false, false, ['--arrival']);
  try {
    await page.goto(`${owner.url}/ui/?mode=fleet`);
    await page.locator('#fleet-key').fill(owner.token);
    await expect(page.locator('#fleet-arrival')).toContainText('pinky1 · N16 · completed-return');
    await page.locator('#fleet-recovery-note').fill('N16 정차 확인');
    await expect(page.locator('#fleet-confirm-arrival')).toBeDisabled();
    await page.locator('#fleet-recovery-confirm').check();
    await page.locator('#fleet-confirm-arrival').click();
    await expect(page.locator('#fleet-arrival option')).toHaveCount(0);
    await expect(page.locator('#fleet-enable')).toBeDisabled();
    await expect(page.locator('#fleet-recovery-confirm')).not.toBeChecked();
    await page.locator('#fleet-recovery-confirm').check();
    await page.locator('#fleet-release').click();
    await expect(page.locator('#fleet-state')).toHaveText('요청 처리 시작 대기');
    await page.locator('#fleet-enable').click();
    await expect(page.locator('#fleet-state')).toHaveText('요청 처리 중');
  } finally { await owner.close(); }
});

test('expired STOP recovery preserves the unresolved command and HOLD', async ({page}) => {
  const owner = await fixture('simulation', false, true, ['--stop']);
  try {
    await page.goto(`${owner.url}/ui/?mode=fleet`);
    await page.locator('#fleet-key').fill(owner.token);
    await page.locator('#fleet-recovery-note').fill('새 기동의 정차 및 유휴 상태 확인');
    await page.locator('#fleet-recovery-confirm').check();
    await page.locator('#fleet-recover-stop').click();
    await expect(page.locator('#fleet-feedback')).toContainText('접수 완료');
    const snapshot = await (await page.request.get(`${owner.url}/api/snapshot?mode=fleet`)).json();
    const command = snapshot.fleet.commands['held-command'];
    expect(command.manual_stop.boot_version).toBe('pinky1-test-boot');
    expect(snapshot.fleet.enabled).toBe(false);
    await expect(page.locator('#fleet-release')).toBeDisabled();
    await expect(page.locator('#fleet-enable')).toBeDisabled();
  } finally { await owner.close(); }
});
