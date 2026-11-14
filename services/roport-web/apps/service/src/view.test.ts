// 실행(CI와 동일): services/roport-web 에서 `node --test 'apps/service/src/**/*.test.ts'`
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { splitLocations, progressIndex, boardingNotice, hhmm } from './view.ts';

const loc = (locationId: string, name: string) => ({ locationId, name });

test('splitLocations: Gate N은 번호순 게이트 줄, 나머지는 아래 줄', () => {
  const r = splitLocations([loc('a', '입국장'), loc('g10', 'Gate 10'), loc('g2', 'Gate 2'), loc('g1', 'gate1')]);
  assert.deepEqual(r.gates.map((l) => l.locationId), ['g1', 'g2', 'g10']);
  assert.deepEqual(r.others.map((l) => l.locationId), ['a']);
});
test('splitLocations: 모든 위치가 빠짐없이 한 번씩 들어간다', () => {
  const list = [loc('x', 'Gate A'), loc('y', '라운지'), loc('z', 'Gate 3')];
  const r = splitLocations(list);
  assert.equal(r.gates.length + r.others.length, list.length);
  assert.deepEqual(r.others.map((l) => l.locationId), ['x', 'y']);
});

test('progressIndex: REQUESTED만 접수 단계, 모르는 상태는 -1', () => {
  assert.equal(progressIndex('REQUESTED'), 0);
  assert.equal(progressIndex('ARRIVED'), -1);
});

test('hhmm: 한국 시간 24시 표기', () => {
  assert.equal(hhmm('2026-11-14T15:05:00Z'), '00:05');
});

const flight = { boardingStart: '2026-11-14T03:00:00Z', boardingEnd: '2026-11-14T03:40:00Z' };
test('boardingNotice: 탑승 전 남은 시간과 30분 이내 경고', () => {
  assert.deepEqual(boardingNotice(flight, new Date('2026-11-14T01:15:00Z')), { tone: 'info', text: '탑승 시작까지 1시간 45분 남았습니다.' });
  assert.deepEqual(boardingNotice(flight, new Date('2026-11-14T01:00:00Z')), { tone: 'info', text: '탑승 시작까지 2시간 남았습니다.' });
  assert.deepEqual(boardingNotice(flight, new Date('2026-11-14T02:40:00Z')), { tone: 'warn', text: '탑승 시작까지 20분 남았습니다.' });
});
test('boardingNotice: 탑승 중·마감', () => {
  assert.equal(boardingNotice(flight, new Date('2026-11-14T03:10:00Z')).text, '탑승 중입니다. 12:40에 마감됩니다.');
  assert.equal(boardingNotice(flight, new Date('2026-11-14T03:40:00Z')).tone, 'done');
});
