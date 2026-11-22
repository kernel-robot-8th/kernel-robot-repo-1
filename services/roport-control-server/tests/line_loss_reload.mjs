// Driven by test_d660_line_loss_keeps_evidence over the same loopback fleet.
import { chromium, expect } from '@playwright/test';
import {readFile, writeFile} from 'node:fs/promises';
const [url, image, runDir] = process.argv.slice(2);
const browser = await chromium.launch({args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader']});
const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
page.setDefaultTimeout(20000);
try {
  await page.goto(url + '/ui/?mode=fleet', {waitUntil:'domcontentloaded'});
  const marker = page.locator('#labels [data-robot=pinky1]');
  for (const reload of [false, true]) {
    if (reload) await page.reload({waitUntil:'domcontentloaded'});
    await expect(page.locator("body")).toHaveAttribute("data-ready", "true", {timeout:20000});
    await expect(marker).toBeVisible();
    await expect(marker).toHaveClass(/stale/);
    await expect(marker).toContainText('마지막 위치');
    await expect(page.locator('#fleet-robot-observations')).toContainText('N10 → N18 도착 미확인');
    await expect(page.locator('#fleet-robot-observations')).toContainText('line_lost');
    await expect(page.locator('#fleet-reservation-owners')).toContainText('edge:N10-N18 → pinky1');
  }
  await page.screenshot({path: image});
  const snapshot = async () => (await (await page.request.get(url + '/api/snapshot?mode=fleet')).json()).fleet;
  const held = await snapshot();
  const cid = Object.keys(held.commands)[0];
  const operatorKey = (await readFile(runDir + '/ui-key.txt', 'utf8')).trim();
  await page.locator('#fleet-key').fill(operatorKey);
  await page.locator('#fleet-recovery-command').selectOption(cid);
  await page.locator('#fleet-recovery-outcome').selectOption('repositioned_n16');
  await page.locator('#fleet-recovery-note').fill('시험 · N16 정차선, N14 방향과 빈 통로 확인');
  await expect(page.locator('#fleet-reconcile')).toBeDisabled();
  // A mistaken operator attestation cannot replace the robot's independent placement report.
  await page.locator('#fleet-recovery-confirm').check();
  await page.locator('#fleet-reconcile').click();
  await expect(page.locator('#fleet-feedback')).toContainText('최신 정지·유휴 상태');
  expect((await snapshot()).resources).toEqual(held.resources);
  await expect(page.locator('#fleet-release')).toBeDisabled();

  const run = JSON.parse(await readFile(runDir + '/run.json', 'utf8'));
  expect(run.simulation).toBe(true);
  process.kill(run.pids.pinky1, 'SIGUSR1');
  await expect.poll(async () => (await snapshot()).robots.pinky1.state.node).toBe('N16');
  await expect(page.locator('#fleet-recovery-confirm')).not.toBeChecked();
  expect((await snapshot()).resources).toEqual(held.resources);
  await page.locator('#fleet-recovery-confirm').check();
  await page.locator('#fleet-reconcile').click();
  await expect(page.locator('#fleet-recovery-command option')).toHaveCount(0);
  await expect(page.locator('#fleet-enable')).toBeDisabled();
  await page.locator('#fleet-recovery-confirm').check();
  await page.locator('#fleet-release').click();
  await expect(page.locator('#fleet-state')).toHaveText('요청 처리 시작 대기');
  await expect(page.locator('#fleet-enable')).toBeEnabled();
  await expect(page.locator('#fleet-robot-observations')).toContainText('N16');
  const before = await snapshot();
  const robot = page.locator('#fleet-robot-observations li').filter({hasText:'pinky1'});
  await expect(robot).toContainText('N16 · line · 정지 관측');
  await expect(robot).toContainText('기체 명령: 없음');
  expect(before.enabled).toBe(false);
  expect(before.commands).toEqual({});
  await writeFile(image.replace('.png', '-standby.json'), JSON.stringify({
    status: await page.locator('#fleet-state').innerText(),
    robot: await robot.innerText(),
    startButton: await page.locator('#fleet-enable').innerText(),
    commandId: cid,
  }, null, 2));
} catch (error) {
  console.error(await page.locator('#fleet-state, #fleet-feedback, #fleet-robot-observations').allTextContents());
  console.error(error);
  throw error;
} finally { await browser.close(); }
