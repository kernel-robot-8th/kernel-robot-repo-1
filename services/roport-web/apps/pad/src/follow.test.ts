import assert from 'node:assert/strict';
import test from 'node:test';
import { framePath, newerFollowStatus, parseFollowStatus, parseReceiptIntent, receiptIntent, validBox } from './follow.ts';
import { followView } from './flow.ts';

const receiptId = 'receipt-test';
const status = {
  receiptId, mode: 'appearance', stage: 'follow_register', status: 'RUNNING', reason: null,
  updatedAt: '2026-11-24T10:00:00Z', revision: 3, allowedActions: ['capture', 'select'], stale: false,
  registration: { frameRef: 'frame-test', imagePath: framePath(receiptId, 'frame-test'), expiresAt: '2026-11-24T10:01:00Z',
    perceptionBootId: 'boot-test', candidates: [{ candidateId: 'person-1', box: [.1,.1,.5,.9] }] },
};

test('receipt selection is atomic and unchanged when restored for retry', () => {
  for (const mode of ['none', 'marker', 'appearance'] as const) {
    const intent = receiptIntent('verification-test', mode, 35);
    assert.deepEqual(parseReceiptIntent(JSON.parse(JSON.stringify(intent))), intent);
    assert.equal(intent.followCard, mode === 'marker' ? 35 : undefined);
  }
  assert.equal(parseReceiptIntent({ ...receiptIntent('verification-test', 'marker', 35), confirmationRequestId: 'different' }), null);
});

test('marker receipt rejects missing or out-of-range cards before submission', () => {
  for (const card of [undefined, 34, 50, 35.5, NaN]) assert.throws(() => receiptIntent('verification-test', 'marker', card));
  assert.throws(() => receiptIntent('', 'none'));
});

test('follow state validates cart session identity and revision', () => {
  assert.equal(parseFollowStatus(status, receiptId)?.stage, 'follow_register');
  for (const patch of [{ receiptId: 'other' }, { revision: -1 }, { revision: 1.5 }, { mode: 'both' }, { stale: undefined }, { allowedActions: ['start'] }]) {
    assert.equal(parseFollowStatus({ ...status, ...patch }, receiptId), null);
  }
});

test('registration cannot request external images or a different session/frame', () => {
  for (const imagePath of ['https://camera.example/image.jpg', '//camera/image.jpg', '/cart-api/follow/other/frames/frame-test', '/shopping/frame']) {
    assert.equal(parseFollowStatus({ ...status, registration: { ...status.registration, imagePath } }, receiptId), null);
  }
  assert.equal(framePath('r/id', 'f id'), '/cart-api/follow/r%2Fid/frames/f%20id');
});

test('registration rejects malformed candidate boxes and expired-format timestamps', () => {
  for (const box of [[0,0,1,2], [0,0,0,1], [1,0,0,1], [0,0,NaN,1], [0,0,1]]) assert.equal(validBox(box), false);
  assert.equal(validBox([0,0,1,1]), true);
  assert.equal(parseFollowStatus({ ...status, registration: { ...status.registration, expiresAt: 'invalid' } }, receiptId), null);
});

test('a late GET response cannot restore an older registration or stage', () => {
  const old = parseFollowStatus(status, receiptId)!;
  const latest = parseFollowStatus({ ...status, revision: 5, stage: 'following', registration: null }, receiptId)!;
  assert.equal(newerFollowStatus(latest, old), latest);
  assert.equal(newerFollowStatus(old, latest), latest);
});

test('HTTP accepted/start preparation is not presented as actual following', () => {
  assert.match(followView('follow_pending', 'RUNNING', 'marker').title, /승인/);
  assert.match(followView('follow_start', 'RUNNING', 'appearance').title, /확인/);
  assert.match(followView('following', 'RUNNING', 'appearance').title, /따라가고/);
  assert.match(followView('following', 'HOLD', 'appearance').title, /확인/);
  assert.match(followView('follow_pending', 'RUNNING', 'none').title, /수령이 완료/);
});


test('completed registration restores target state without retired capture fields', () => {
  const registration = { registered: true, expiresAt: '2026-11-24T10:01:00Z', perceptionBootId: 'boot-test' };
  const result = parseFollowStatus({ ...status, stage: 'follow_pending', registration }, receiptId);
  assert.deepEqual(result?.registration, registration);
  assert.equal(result?.registration?.frameRef, undefined);
  assert.equal(parseFollowStatus({ ...status, registration: { ...registration, registered: false } }, receiptId), null);
});
