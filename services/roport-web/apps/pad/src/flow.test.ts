// 실행(CI와 동일): services/roport-web 에서 `node --test 'apps/*/src/**/*.test.ts'`
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { bigButton, initialState, movingPhase, reduce, type Action, type MovingStep, type PadState } from './flow.ts';
import type { VerifiedPickup } from './types';

const pickup: VerifiedPickup = {
  pickupId: 'p1',
  reservationNo: 'ARV9H3MP',
  orderNo: 'DEV-0002',
  customerName: '테스트 고객 1',
  location: { locationId: 'dev-loc-arrival', name: '입국장' },
  flight: { flightNo: 'ZZ303', gateCode: 'G3', boardingStart: '2026-11-14T03:00:00Z', boardingEnd: '2026-11-14T03:40:00Z' },
  items: [{ itemId: 'i1', productName: '샘플 초콜릿 C', quantity: 3 }],
  packageCount: 1,
};
const run = (actions: Action[], from: PadState = initialState) => actions.reduce(reduce, from);
const moving = (s: PadState) => {
  assert.equal(s.step.name, 'moving');
  return s.step as MovingStep;
};
const startedFollow = () =>
  run([
    { type: 'go', step: { name: 'chooseService', pickup } },
    { type: 'send', token: 1 },
    { type: 'serviceAnswered', token: 1, pickup, mode: 'FOLLOW', res: { accepted: true } },
    { type: 'robotState', motion: 'MOVING' },
  ]);

// ── 리뷰 1: 화면 전환 뒤 늦게 온 응답 무시 ──
test('QR 확인 중 처음부터를 누르면 늦게 온 확인 결과가 화면을 덮지 않는다', () => {
  const s = run([
    { type: 'send', token: 1, step: { name: 'verifying' } },
    { type: 'reset' },
    { type: 'verified', token: 1, result: { ok: true, pickup } },
  ]);
  assert.deepEqual(s.step, { name: 'idle' });
  assert.equal(s.request, null);
});

test('추종 중 중지 직후 추종 종료를 누르면 늦게 온 중지 응답이 이동 화면으로 되돌리지 않는다', () => {
  const s = run([
    { type: 'send', token: 2 },
    { type: 'go', step: { name: 'receipt', pickup } },
    { type: 'commandAnswered', token: 2, command: 'PAUSE', res: { accepted: true } },
  ], startedFollow());
  assert.equal(s.step.name, 'receipt');
});

test('출발 요청 중 화면을 떠나면 늦게 온 출발 접수가 이동 화면을 열지 않는다', () => {
  const s = run([
    { type: 'go', step: { name: 'chooseService', pickup } },
    { type: 'send', token: 1 },
    { type: 'reset' },
    { type: 'serviceAnswered', token: 1, pickup, mode: 'GATE_GUIDE', res: { accepted: true } },
  ]);
  assert.deepEqual(s.step, { name: 'idle' });
});

test('응답을 기다리는 동안 두 번째 요청은 받지 않는다', () => {
  const s = run([{ type: 'send', token: 1 }, { type: 'send', token: 2 }]);
  assert.equal(s.request, 1);
});

// ── 리뷰 2: 접수와 실제 상태 구분 ──
test('출발 접수만으로는 주행 중으로 표시하지 않는다', () => {
  const s = run([
    { type: 'go', step: { name: 'chooseService', pickup } },
    { type: 'send', token: 1 },
    { type: 'serviceAnswered', token: 1, pickup, mode: 'GATE_GUIDE', res: { accepted: true } },
  ]);
  assert.equal(movingPhase(moving(s)), 'START_PENDING');
  assert.equal(movingPhase(moving(reduce(s, { type: 'robotState', motion: 'MOVING' }))), 'MOVING');
});

test('중지 접수는 확인 대기이고 카트 정지 보고 후에만 정지·재개 버튼으로 바뀐다', () => {
  const pending = run([
    { type: 'send', token: 2 },
    { type: 'commandAnswered', token: 2, command: 'PAUSE', res: { accepted: true } },
  ], startedFollow());
  assert.equal(movingPhase(moving(pending)), 'PAUSE_PENDING');
  assert.equal(bigButton('PAUSE_PENDING'), 'PAUSE'); // 멈춘 것이 확인될 때까지 중지를 다시 누를 수 있다

  // 아직 움직인다는 보고는 중지 대기를 지우지 않는다.
  assert.equal(movingPhase(moving(reduce(pending, { type: 'robotState', motion: 'MOVING' }))), 'PAUSE_PENDING');

  const paused = reduce(pending, { type: 'robotState', motion: 'PAUSED' });
  assert.equal(movingPhase(moving(paused)), 'PAUSED');
  assert.equal(bigButton('PAUSED'), 'RESUME');
});

test('재개 접수는 확인 대기이고 주행 보고 후에만 주행 중이 된다', () => {
  const paused = run([
    { type: 'send', token: 2 },
    { type: 'commandAnswered', token: 2, command: 'PAUSE', res: { accepted: true } },
    { type: 'robotState', motion: 'PAUSED' },
    { type: 'send', token: 3 },
    { type: 'commandAnswered', token: 3, command: 'RESUME', res: { accepted: true } },
  ], startedFollow());
  assert.equal(movingPhase(moving(paused)), 'RESUME_PENDING');
  assert.equal(bigButton('RESUME_PENDING'), 'PAUSE'); // 다시 움직일 수 있는 동안에는 중지 버튼
  assert.equal(movingPhase(moving(reduce(paused, { type: 'robotState', motion: 'MOVING' }))), 'MOVING');
});

test('출발 확인 중에도 중지 버튼은 누를 수 있다', () => {
  assert.equal(bigButton('START_PENDING'), 'PAUSE');
  assert.equal(bigButton('ARRIVED'), null);
});

test('중지는 다른 요청을 기다리는 중에도 보낼 수 있고, 앞 요청의 응답은 버린다', () => {
  const s = run([
    { type: 'send', token: 2 },
    { type: 'commandAnswered', token: 2, command: 'PAUSE', res: { accepted: true } },
    { type: 'robotState', motion: 'PAUSED' },
    { type: 'send', token: 3 }, // 재개 요청 중
    { type: 'send', token: 4, stop: true }, // 중지가 앞선다
    { type: 'commandAnswered', token: 3, command: 'RESUME', res: { accepted: true } },
    { type: 'commandAnswered', token: 4, command: 'PAUSE', res: { accepted: true } },
  ], startedFollow());
  assert.equal(movingPhase(moving(s)), 'PAUSE_PENDING');
  assert.equal(s.request, null);
});

test('중지 응답을 기다리는 중의 중복 중지는 받지 않는다', () => {
  const s = run([{ type: 'send', token: 2, stop: true }, { type: 'send', token: 3, stop: true }], startedFollow());
  assert.equal(s.request, 2);
});

test('거절·통신 실패는 상태를 바꾸지 않고 안내만 띄운다', () => {
  const before = startedFollow();
  const rejected = run([{ type: 'send', token: 2 }, { type: 'commandAnswered', token: 2, command: 'PAUSE', res: { accepted: false } }], before);
  assert.equal(movingPhase(moving(rejected)), 'MOVING');
  assert.equal(rejected.notice, 'COMMAND_REJECTED');
  const failed = run([{ type: 'send', token: 2 }, { type: 'commandAnswered', token: 2, command: 'PAUSE', res: null }], before);
  assert.equal(movingPhase(moving(failed)), 'MOVING');
  assert.equal(failed.notice, 'ERROR');
});

test('수령 확인 응답이 와야 복귀 화면으로 간다', () => {
  const s = run([
    { type: 'go', step: { name: 'receipt', pickup } },
    { type: 'send', token: 1 },
  ]);
  assert.equal(s.step.name, 'receipt');
  assert.equal(reduce(s, { type: 'receiptAnswered', token: 1, res: { accepted: true } }).step.name, 'returning');
});
