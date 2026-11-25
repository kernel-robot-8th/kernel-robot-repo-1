import { test, expect } from '@playwright/test';
import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';

test.skip(!process.env.ROPORT_TEST_DATABASE_URL, 'Requires isolated PostgreSQL test database');

async function fixture() {
  const child = spawn('.venv/bin/python', ['-u', 'tests/fleet_web_fixture.py', 'simulation', '--registry'], {stdio: ['ignore', 'pipe', 'pipe']});
  const lines = createInterface({input: child.stdout});
  let diagnostics = '';
  child.stderr.on('data', data => { diagnostics += data; });
  const close = async () => {
    lines.close(); child.kill('SIGINT');
    await new Promise(resolve => child.exitCode !== null ? resolve() : child.once('exit', resolve));
  };
  try {
    const ready = await new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error(`registry fixture timeout: ${diagnostics}`)), 15000);
      lines.once('line', line => { clearTimeout(timer); resolve(JSON.parse(line)); });
      child.once('exit', code => { clearTimeout(timer); reject(new Error(`registry fixture ${code}: ${diagnostics}`)); });
    });
    return {...ready, close};
  } catch (error) { await close(); throw error; }
}

async function open(page, owner) {
  await page.goto(`${owner.url}/ui/?mode=fleet`);
  await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
  await page.locator('#fleet-key').fill(owner.token);
  await page.locator('[data-tab=pinky]').click();
  await expect(page.locator('#ps-save')).toBeEnabled();
}

async function fill(page, index) {
  await page.locator('#ps-slot').selectOption(`pinky${index}`);
  await page.locator('#ps-device').fill(`sim-browser-${index}`);
  await page.locator('#ps-ip').fill(`192.0.2.${10 + index}`);
  await page.locator('#ps-domain').fill(String(13 + index));
  await page.locator('#ps-role').selectOption(index === 3 ? 'smartcart' : 'logistics');
  await page.locator('#ps-calib').fill(['pinky_d660', 'pinky_6422', 'pinky_764e'][index - 1]);
}

test('three fixed slots register, change role, exclude and explicitly unregister through the real owner and database', async ({page}, testInfo) => {
  const owner = await fixture(), errors = [];
  page.on('pageerror', error => errors.push(error.message));
  try {
    await open(page, owner);
    await expect(page.locator('#ps-registry-rows tr')).toHaveCount(3);
    await expect(page.locator('#ps-slot option')).toHaveCount(3);
    await expect(page.locator('#fleet-assign')).toBeDisabled();
    await expect(page.getByRole('button', {name: '업데이트 설치', exact: true})).toHaveCount(0);
    for (const index of [1, 2, 3]) {
      await fill(page, index);
      await page.locator('#ps-save').click();
      await expect(page.locator(`#ps-registry-rows [data-slot=pinky${index}]`)).toContainText(`sim-browser-${index}`);
      await expect(page.locator('#ps-msg')).toContainText('저장 완료');
    }
    await page.locator('#pinky').screenshot({path: testInfo.outputPath('registry-ui.png')});
    await page.locator('#ps-slot').selectOption('pinky2');
    await expect(page.locator('#ps-device')).toBeDisabled();
    await page.locator('#ps-role').selectOption('smartcart');
    await page.locator('#ps-save').click();
    await expect(page.locator('#ps-registry-rows [data-slot=pinky2]')).toContainText('스마트카트');
    await expect(page.locator('#fleet-roles')).toContainText('pinky2: 스마트카트');
    await page.locator('#ps-exclude').click();
    await expect(page.locator('#ps-exclude')).toHaveText('배정에 다시 포함');
    await expect(page.locator('#fleet-roles')).toContainText('pinky2: 배정 제외');
    await page.locator('#ps-exclude').click();
    await expect(page.locator('#ps-exclude')).toHaveText('배정에서 제외');
    await expect(page.locator('#fleet-roles')).toContainText('pinky2: 스마트카트');
    await expect(page.locator('#ps-unregister')).toBeDisabled();
    await page.locator('#ps-exclude').click();
    await expect(page.locator('#ps-exclude')).toHaveText('배정에 다시 포함');
    await expect(page.locator('#ps-unregister')).toBeDisabled();
    await page.locator('#ps-empty-load').check();
    await expect(page.locator('#ps-unregister')).toBeDisabled();
    await page.locator('#ps-off-lane').check();
    await expect(page.locator('#ps-unregister')).toBeEnabled();
    await page.locator('#ps-unregister').click();
    await expect(page.locator('#ps-registry-rows [data-slot=pinky2]')).toContainText('미등록');
    await expect(page.locator('#ps-registry-rows tr')).toHaveCount(3);
    const snapshot = (await (await page.request.get(`${owner.url}/api/snapshot?mode=fleet`)).json()).fleet;
    expect(snapshot.registry.robots.filter(r => r.kind === 'pinky' && r.registered)).toHaveLength(2);
    expect(snapshot.roles.pinky2).toBeNull();
    expect(errors).toEqual([]);
  } finally { await owner.close(); }
});

test('a committed registration with a lost HTTP reply retries the same request instead of creating another binding', async ({page}) => {
  const owner = await fixture(), requests = [];
  try {
    await open(page, owner); await fill(page, 1);
    await page.route('**/api/fleet/commands', async route => {
      requests.push(route.request().postDataJSON());
      if (requests.length === 1) {
        const response = await route.fetch();
        expect(response.status()).toBe(200);
        await route.fulfill({status: 502, contentType: 'text/html', body: '<h1>Bad Gateway</h1>'});
      } else await route.continue();
    });
    await page.locator('#ps-save').click();
    await expect(page.locator('#ps-msg')).toContainText('HTTP 502');
    await expect(page.locator('#ps-registry-rows [data-slot=pinky1]')).toContainText('sim-browser-1');
    await page.locator('#ps-retry').click();
    await expect(page.locator('#ps-msg')).toContainText('저장 완료');
    expect(requests[1].request).toEqual(requests[0].request);
    const snapshot = (await (await page.request.get(`${owner.url}/api/snapshot?mode=fleet`)).json()).fleet;
    const row = snapshot.registry.robots.find(r => r.robot_id === 'pinky1');
    expect(row.revision).toBe(1);
    expect(snapshot.roles.pinky1).toBe('logistics');
    await page.reload();
    await page.locator('[data-tab=pinky]').click();
    await expect(page.locator('#ps-save')).toBeDisabled();
    await expect(page.locator('#ps-registry-rows [data-slot=pinky1]')).toContainText('sim-browser-1');
  } finally { await owner.close(); }
});
