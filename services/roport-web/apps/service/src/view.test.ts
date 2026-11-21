// 실행(CI와 동일): services/roport-web 에서 `node --test 'apps/service/src/**/*.test.ts'`
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { splitLocations, progressIndex, boardingNotice, hhmm, when, mmss, qrDeadline, qrSecondsLeft } from './view.ts';

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

test('when: 오늘(KST)은 시각만, 다른 날은 날짜와 함께', () => {
  const now = new Date('2026-11-14T13:00:00Z'); // KST 11/14 22:00
  assert.equal(when('2026-11-14T14:30:00Z', now), '23:30');
  assert.equal(when('2026-11-14T15:00:00Z', now), '11/15 00:00'); // KST 자정 넘김
  assert.equal(when('2025-11-14T03:00:00Z', now), '2025/11/14 12:00'); // 작년 같은 날은 오늘이 아니다
  assert.equal(when('2026-12-31T15:30:00Z', now), '2027/1/1 00:30'); // KST로 해가 바뀜
});

test('qrDeadline: 서버 유효 기간을 받은 순간부터 센다(휴대폰 시계가 서버와 달라도 같음)', () => {
  const received = Date.parse('2026-11-21T03:20:00Z'); // 휴대폰 시계가 서버보다 20분 빠른 경우
  assert.equal(qrDeadline('2026-11-21T03:00:00Z', '2026-11-21T03:10:00Z', received), received + 600_000);
  // 해석할 수 없거나 기간이 0 이하이면 받은 즉시 만료
  assert.equal(qrDeadline('x', '2026-11-21T03:10:00Z', received), received);
  assert.equal(qrDeadline('2026-11-21T03:10:00Z', '2026-11-21T03:00:00Z', received), received);
});

test('qrSecondsLeft: 만료까지 남은 초(올림), 지났으면 0', () => {
  const deadline = Date.parse('2026-11-21T03:10:00Z');
  assert.equal(qrSecondsLeft(deadline, deadline - 600_000), 600);
  assert.equal(qrSecondsLeft(deadline, deadline - 200), 1);
  assert.equal(qrSecondsLeft(deadline, deadline), 0);
  assert.equal(qrSecondsLeft(deadline, deadline + 60_000), 0);
});

test('mmss: 분:초 두 자리', () => {
  assert.equal(mmss(600), '10:00');
  assert.equal(mmss(581), '9:41');
  assert.equal(mmss(5), '0:05');
  assert.equal(mmss(-3), '0:00');
});
