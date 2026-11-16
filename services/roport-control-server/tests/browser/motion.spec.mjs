import { test, expect } from '@playwright/test';
import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';
import { readFile } from 'node:fs/promises';

test('virtual driver motion reaches original UI; Nav2 and stale samples remain visible', async ({page}) => {
  const child = spawn('.venv/bin/python', ['-u', 'tools/run_virtual_trial.py', '--scenario', 'single'], {stdio:['ignore','pipe','pipe']});
  let diagnostics = '';
  child.stderr.on('data', d => diagnostics += d);
  const lines = createInterface({input: child.stdout});
  try {
    const info = await new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error(diagnostics)), 20000);
      child.once('exit', code => { clearTimeout(timer); reject(new Error(`launcher ${code}: ${diagnostics}`)); });
      lines.once('line', line => { clearTimeout(timer); resolve(JSON.parse(line)); });
    });
    const url = new URL(info.url).origin;
    const key = (await readFile(info.key_file, 'utf8')).trim();
    const errors = [];
    page.on('pageerror', e => errors.push(e.message));
    await page.goto(info.url);
    await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
    const label = page.locator('[data-robot="pinky1"]');
    await expect(label).toBeVisible();
    await expect(page.locator('#telemetry')).toContainText('가상 센서 위치');
    const snapshot = async () => (await (await page.request.get(`${url}/api/snapshot?mode=trial`)).json());
    const first = (await snapshot()).robots[0];
    await page.locator('#trial-key').fill(key);
    await page.locator('[data-action="start"]').click();
    await expect.poll(async () => {
      const row = (await snapshot()).robots[0];
      return row.pose_live && Math.hypot(row.pose[0]-first.pose[0], row.pose[1]-first.pose[1]) > 0.01;
    }).toBeTruthy();
    const moved = (await snapshot()).robots[0];
    expect(moved.status.node_id).toBe(first.status.node_id); // Movement between nodes, not a node jump.
    await expect(label).toBeVisible();

    // Projection checks use a controlled stream, without changing the running virtual robot.
    const frame = await snapshot();
    let stream;
    await page.routeWebSocket('**/api/state?*', ws => { stream=ws; ws.send(JSON.stringify(frame)); });
    await page.reload();
    await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
    await expect.poll(() => Boolean(stream)).toBeTruthy();
    const row = frame.robots[0];
    frame.revision += 1;
    row.mode = 'nav2 · 노드 미확인';
    row.status.mode = 'nav2'; row.status.node_id = null;
    row.pose_key = row.pose_key.split(':')[0] + ':999';
    row.pose = [row.pose[0] + 0.1, row.pose[1], 0, row.pose[3] + 0.1];
    stream.send(JSON.stringify(frame));
    await expect(label).toBeVisible();
    await expect(page.locator('#telemetry')).toContainText(row.pose[0].toFixed(2));
    frame.revision += 1;
    row.pose_stale = true; row.pose_live = false; row.pose_age = 4;
    row.pose_source += ' · 마지막 확인 위치';
    stream.send(JSON.stringify(frame));
    await expect(label).toBeVisible();
    await expect(label).toContainText('마지막 위치');
    await expect(page.locator('#telemetry')).toContainText('4.0초 · 갱신 지연');
    frame.revision += 1;
    frame.trial.motion_diagnostics = {rejected: 3, reasons: {clock_skew_or_expired: 3}};
    stream.send(JSON.stringify(frame));
    await expect(page.locator('#trial-motion-diagnostics')).toContainText('위치 수신 거부 누적 3건');
    await expect(page.locator('#trial-motion-diagnostics')).toContainText('시계 동기화 확인');
    frame.revision += 1;
    frame.trial.motion_diagnostics = {rejected: 0, reasons: {}};
    stream.send(JSON.stringify(frame));
    await expect(page.locator('#trial-motion-diagnostics')).toBeHidden();
    expect(errors).toEqual([]);
  } finally {
    lines.close();
    child.kill('SIGINT');
    if (child.exitCode === null) await new Promise(resolve => child.once('exit', resolve));
  }
});
