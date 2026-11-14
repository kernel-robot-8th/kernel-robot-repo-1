import { test, expect } from '@playwright/test';
import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';
import { mkdtemp, readFile, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

test('virtual reset restores position after driving or HOLD and preserves both runs', async ({page}, testInfo) => {
  const output = await mkdtemp(join(tmpdir(), 'roport-reset-'));
  const child = spawn('.venv/bin/python', ['-u', 'tools/run_virtual_trial.py', '--scenario', 'single', '--output', join(output, 'run')], {stdio:['ignore','pipe','pipe']});
  let diagnostics = '';
  child.stderr.on('data', d => diagnostics += d);
  const lines = createInterface({input: child.stdout});
  try {
    const info = await new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error(diagnostics)), 20000);
      child.once('exit', code => { clearTimeout(timer); reject(new Error(`launcher ${code}: ${diagnostics}`)); });
      lines.once('line', line => { clearTimeout(timer); resolve(JSON.parse(line)); });
    });
    const origin = new URL(info.url).origin;
    const key = (await readFile(info.key_file, 'utf8')).trim();
    const snapshot = async () => (await (await page.request.get(`${origin}/api/snapshot?mode=trial`)).json());
    await page.goto(info.url);
    await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
    const reset = page.locator('[data-action="reset"]');
    await expect(reset).toBeVisible();
    await expect(reset).toBeDisabled();
    await page.locator('#trial-key').fill(key);
    await expect(reset).toBeEnabled();
    await expect.poll(async () => (await snapshot()).robots[0].pose_live).toBeTruthy();
    const initial = (await snapshot()).robots[0].pose;
    let previousArchive;
    for (const hold of [false, true]) {
      const before = (await snapshot()).trial;
      await page.locator('[data-action="start"]').click();
      await expect.poll(async () => {
        const row = (await snapshot()).robots[0];
        return row.pose_live && Math.hypot(row.pose[0]-initial[0], row.pose[1]-initial[1]) > 0.015;
      }).toBeTruthy();
      if (hold) {
        await page.locator('[data-action="stop"]').click();
        await expect(page.locator('#trial-phase')).toHaveText('시험 보류');
      }
      page.once('dialog', dialog => dialog.accept());
      await reset.click();
      await expect.poll(async () => {
        try {
          const current = await snapshot();
          return current.trial?.session_id !== before.session_id && current.trial?.can_start && current.robots[0].pose_live;
        } catch { return false; }
      }, {timeout:30000}).toBeTruthy();
      await expect(page.locator('[data-action="start"]')).toBeEnabled({timeout:10000});
      await expect(page.locator('#trial-phase')).toHaveText('출발 대기');
      expect(page.url()).toBe(info.url);
      const fresh = await snapshot();
      expect(fresh.trial.task_id).not.toBe(before.task_id);
      expect(fresh.robots[0].status.node_id).toBe('N16');
      expect(fresh.robots[0].pose[0]).toBeCloseTo(initial[0], 3);
      expect(fresh.robots[0].pose[1]).toBeCloseTo(initial[1], 3);
      const run = JSON.parse(await readFile(`${info.run_dir}/run.json`, 'utf8'));
      expect(run.previous_run_dir).not.toBe(previousArchive);
      const old = JSON.parse(await readFile(`${run.previous_run_dir}/control.json`, 'utf8'));
      expect(old.task_id).toBe(before.task_id);
      expect(Object.keys(old.events).length).toBeGreaterThan(0);
      if (previousArchive) expect(JSON.parse(await readFile(`${previousArchive}/outcome.json`, 'utf8')).result).toBe('reset');
      previousArchive = run.previous_run_dir;
    }
    await page.screenshot({path:testInfo.outputPath('virtual-reset-ready.png'), fullPage:true});
  } finally {
    lines.close();
    child.kill('SIGINT');
    if (child.exitCode === null) await new Promise(resolve => child.once('exit', resolve));
    await rm(output, {recursive:true, force:true});
  }
});
