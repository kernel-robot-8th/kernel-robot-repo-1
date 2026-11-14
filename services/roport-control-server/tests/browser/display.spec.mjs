import { test, expect } from '@playwright/test';

for (const exactGeometry of [false, true]) {
test(`measured layout: ${exactGeometry ? 'exact geometry' : 'occupancy fallback'} survives view changes`, async ({ page }) => {
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  await page.route('**/api/bootstrap?*', async route => {
    const response = await route.fetch();
    const data = await response.json();
    data.example_map = false;
    data.map.origin = [-2, -1, 0];
    data.map.wall_height = 0.24;
    data.map.layout = {version: 'fixture-v1', graph_version: 'fixture-lines-v1', mounts: [
      {robot_id: 'omx1', edge_center: [-0.5, 0, 0.24], edge_yaw: 0, yaw: 0},
      {robot_id: 'omx2', edge_center: [0, 0, 0.24], edge_yaw: 0, yaw: Math.PI},
    ], nodes: [
      {id: 'N01', place: 'P4', position: [-1, 0], links: ['N12']},
      {id: 'N12', nav2_switch: true, position: [0, 0], links: ['N01']},
    ]};
    if (exactGeometry) data.map.layout.geometry = [
      {name: 'thin_wall', kind: 'wall', position: [-0.5, 0.2, 0.12], size: [1.5, 0.005, 0.24], yaw: 0.1},
      {name: 'low_box', kind: 'object', position: [-1, -0.5, 0.055], size: [0.5, 0.4, 0.11], yaw: 0.1},
      {name: 'shelf_02', kind: 'object', position: [-0.5, -0.5, 0.12], size: [0.5, 0.21, 0.24], yaw: 0.1},
    ];
    await route.fulfill({response, json: data});
  });
  await page.goto('/ui/');
  await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
  await expect(page.locator('#map-detail')).toContainText('24cm');
  await expect(page.locator('#map-name')).toContainText('실측 fixture-v1');
  await expect(page.locator('.map-note')).toContainText(exactGeometry ? '벽·받침 외곽 표시' : '점유셀 돌출 표시');
  const point = page.locator('[data-node="N01"]');
  await expect(point).toHaveText('P4 · N01');
  await expect(point).toBeVisible();
  const before = await point.boundingBox();
  await page.getByRole('button', {name: '평면', exact: true}).click();
  await expect.poll(async () => (await point.boundingBox()).y).not.toBe(before.y);
  await expect(page.locator('[data-node="N12"]')).toHaveText('N12 · Nav2');
  await expect(page.locator('[data-robot="omx1"]')).toHaveText('OMX 01 · 설치 배치');
  await expect(page.locator('[data-robot="omx2"]')).toBeVisible();
  await page.getByRole('button', {name: 'OMX 01 선택'}).click();
  await expect(page.locator('#telemetry')).toContainText('설치 배치 (기준값)');
  await expect(page.locator('#state')).toContainText('미수신');
  await expect(page.locator('#freshness')).toHaveText('수신 이력 없음');
  await expect(page.locator('#focus')).toBeEnabled();
  await expect(page.locator('#telemetry')).toContainText('미수신 · 기준 자세');
  await expect(page.locator('[data-robot="pinky1"]')).toBeHidden();
  await expect(page.locator('[data-robot="omx3"]')).toBeHidden();
  expect(errors).toEqual([]);
});
}

test('OFFLINE loads local 3D assets and retains detail tabs', async ({ page }) => {
  const errors = [];
  page.on('pageerror', (error) => errors.push(error.message));
  await page.goto('/ui/');
  await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
  await expect(page.locator('#mode')).toContainText('OFFLINE');
  await expect(page.locator('#source')).toContainText('실행부 미연결');
  await expect(page.locator('.robot')).toHaveCount(6);
  await expect(page.locator('canvas')).toHaveCount(2);
  await expect(page.locator('#tasks')).toContainText('수신된 작업이 없습니다');
  await expect(page.getByRole('button', { name: '작업 승인', exact: true })).toBeDisabled();
  await page.getByRole('button', { name: 'OMX 02 선택' }).click();
  await expect(page.locator('#name')).toHaveText('OMX 02');
  await page.getByRole('button', { name: 'OMX', exact: true }).click();
  await expect(page.locator('#omx')).toBeVisible();
  await page.getByRole('button', { name: 'Pinky', exact: true }).click();
  await expect(page.locator('#pinky')).toBeVisible();
  await page.getByRole('button', { name: '통합관제', exact: true }).click();
  await expect(page.locator('#name')).toHaveText('OMX 02');
  await page.getByRole('button', { name: '평면', exact: true }).click();
  await expect(page.locator('[data-view="top"]')).toHaveAttribute('aria-pressed', 'true');
  expect(errors).toEqual([]);
});

test('DEMO playback, stale state, disconnect and fresh reconnect', async ({ page, context }) => {
  const errors = [];
  page.on('pageerror', (error) => errors.push(error.message));
  await page.goto('/ui/?mode=demo');
  await expect(page.locator('body')).toHaveAttribute('data-ready', 'true');
  await expect(page.locator('#source')).toHaveText('모의 상태 수신');
  await expect(page.locator('#mode')).toContainText('DEMO');
  await expect(page.locator('[data-robot="pinky1"]')).toBeVisible();
  await expect(page.locator('[data-robot="omx1"]')).toBeVisible();
  await expect(page.locator('.task')).toHaveCount(3);
  await page.locator('#play').click();
  await expect(page.locator('#play')).toHaveText('동작 재생');
  await page.locator('#drop').click();
  await expect(page.locator('.robot').first()).toHaveAttribute('data-stale', 'true', { timeout: 6000 });
  await page.locator('#drop').click();
  await expect(page.locator('.robot').first()).toHaveAttribute('data-stale', 'false');
  await context.setOffline(true);
  await expect(page.locator('#source')).toContainText('연결 끊김', { timeout: 10000 });
  await expect(page.locator('#play')).toBeDisabled();
  await context.setOffline(false);
  await expect(page.locator('#source')).toHaveText('모의 상태 수신', { timeout: 15000 });
  await expect(page.locator('#play')).toBeEnabled();
  await expect(page.locator('.robot').first()).toHaveAttribute('data-stale', 'false');
  expect(errors).toEqual([]);
});

test('Pinky Settings: OFFLINE is validate-only, DEMO edits stay in the page', async ({ page }) => {
  const errors = [];
  page.on('pageerror', (error) => errors.push(error.message));
  await page.goto('/ui/');
  await page.getByRole('button', { name: 'Pinky', exact: true }).click();
  await expect(page.locator('#pinky-detail')).toContainText('실행부·운영 DB 미연결');
  await expect(page.locator('#pinky-detail')).toContainText('등록된 Pinky가 없습니다');
  await page.locator('#ps-name').fill('pinky1');
  await page.locator('#ps-ip').fill('192.0.2.11');
  await page.getByRole('button', { name: '형식 확인' }).click();
  await expect(page.locator('#ps-msg')).toContainText('저장하지 않음');
  await page.goto('/ui/?mode=demo');
  await page.getByRole('button', { name: 'Pinky', exact: true }).click();
  await expect(page.locator('.ps-table tr')).toHaveCount(4);
  await page.getByRole('button', { name: '릴리즈 설치' }).first().click();
  await expect(page.locator('.ps-jobs')).toContainText('origin/main');
  await page.locator('#ps-name').fill('Bad Name');
  await page.getByRole('button', { name: '저장 (DEMO)' }).click();
  await expect(page.locator('#ps-msg')).toContainText('입력 확인');
  expect(errors).toEqual([]);
});
