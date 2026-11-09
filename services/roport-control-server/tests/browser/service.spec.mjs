import { test, expect } from '@playwright/test';
import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';

test('service: inbound then outbound through the same MQTT owner and PR45 Host', async ({page}, testInfo) => {
  const child = spawn('.venv/bin/python', ['-u','tests/trial_web_fixture.py','line-only','two','all','omx'], {stdio:['ignore','pipe','pipe']});
  const lines = createInterface({input: child.stdout});
  let diagnostics = ''; child.stderr.on('data', d => diagnostics += d);
  try {
    const {url, token} = await new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error(diagnostics || 'fixture timeout')), 20000);
      lines.once('line', s => {clearTimeout(timer); resolve(JSON.parse(s));});
      child.once('exit', code => {clearTimeout(timer); reject(new Error(`fixture ${code}: ${diagnostics}`));});
    });
    const errors = []; page.on('pageerror', e => errors.push(e.message));
    await page.goto(`${url}/ui/`);
    await page.locator('#trial-key').fill(token);
    for (const operation of ['inbound', 'outbound']) {
      await page.locator('#service-operation').selectOption(operation);
      await page.locator('#service-source-confirm').check();
      await page.locator('#service-preview-button').click();
      await expect(page.locator('#service-feedback')).toContainText('계획 확인 완료');
      await expect(page.locator('[data-action="start"]')).toBeDisabled();
      await expect(page.locator('#scenario-editor')).not.toHaveAttribute('open', '');
      await page.locator('#service-apply').click();
      await expect(page.locator('#service-feedback')).toContainText('서비스 요청 접수');
      await expect(page.locator('[data-action="start"]')).toBeEnabled();
      await expect(page.locator('#scenario-fields')).toHaveJSProperty('disabled', true);
      await page.locator('[data-action="start"]').click();
      if (operation === 'inbound') {
        await expect(page.locator('#trial-phase')).toHaveText('창고 상차 완료 대기');
        await page.locator('#trial-dummy').click();
      }
      await expect(page.locator('#service-completion')).toContainText(operation === 'inbound' ? '입고·복귀 시험 완료' : '인도 준비 시험 완료', {timeout:20000});
      await expect(page.locator('#trial-phase')).toHaveText(operation === 'inbound' ? '입고·복귀 시험 완료' : '인도 준비 시험 완료');
      await expect(page.locator('#service-feedback')).toContainText('작업 시험 완료');
      const snapshot = await (await page.request.get(`${url}/api/snapshot?mode=trial`)).json();
      expect(snapshot.trial.phase).toBe('COMPLETE');
      expect(snapshot.trial.transfers.at(-1).completion_source).toBe('mqtt');
      expect(snapshot.trial.transfers.at(-1).placement_confirmed).toBe(null);
      expect(snapshot.trial.plan.steps.every(s => s.role === (operation === 'inbound' ? 'logistics' : 'smartcart'))).toBeTruthy();
    }
    await page.screenshot({path:testInfo.outputPath('service-complete.png'), fullPage:true});
    expect(errors).toEqual([]);
  } finally {
    lines.close(); child.kill('SIGINT');
    await new Promise(resolve => child.exitCode !== null ? resolve() : child.once('exit', resolve));
  }
});
