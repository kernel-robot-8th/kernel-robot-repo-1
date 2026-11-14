// 실행(CI와 동일): services/roport-web 에서 `node --test 'apps/*/src/**/*.test.ts'`
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { newVerificationRequestId, toVerifyResult } from './verify.ts';
import type { VerifiedPickup } from './types';

const pickup: VerifiedPickup = {
  pickupId: 'p1',
  reservationNo: 'GT2X7KQA',
  orderNo: 'DEV-0001',
  customerName: '테스트 고객 1',
  location: { locationId: 'dev-loc-gate2', name: 'Gate 2' },
  flight: { flightNo: 'ZZ202', gateCode: 'G2', boardingStart: '2026-11-14T03:00:00Z', boardingEnd: '2026-11-14T03:30:00Z' },
  items: [{ itemId: 'i1', productName: '샘플 향수 A', quantity: 1 }],
  packageCount: 1,
};

test('확인되면 서버 pickup을 그대로 쓴다(새 확인 201·재전송 200)', () => {
  for (const status of [201, 200]) {
    assert.deepEqual(toVerifyResult(status, { result: 'VERIFIED', reasonCode: null, cartAssignmentChecked: false, pickup }), {
      ok: true,
      pickup,
    });
  }
});

test('거절 사유를 화면 사유로 바꾼다', () => {
  assert.deepEqual(toVerifyResult(201, { result: 'REJECTED', reasonCode: 'ALREADY_RECEIVED', pickup: null }), {
    ok: false,
    reason: 'ALREADY_RECEIVED',
  });
  assert.deepEqual(toVerifyResult(201, { result: 'REJECTED', reasonCode: 'PICKUP_NOT_ACTIVE', pickup: null }), {
    ok: false,
    reason: 'NOT_ACTIVE',
  });
});

test('예약 없음(404)은 고객에게 다시 확인하라고 안내한다', () => {
  assert.deepEqual(toVerifyResult(404, { code: 'RESERVATION_NOT_FOUND' }), { ok: false, reason: 'NOT_FOUND' });
});

test('설정·충돌·형식 오류와 해석할 수 없는 응답은 통신 오류로 둔다', () => {
  assert.equal(toVerifyResult(401, { code: 'CART_UNAUTHENTICATED' }), null);
  assert.equal(toVerifyResult(409, { code: 'IDEMPOTENCY_CONFLICT' }), null);
  assert.equal(toVerifyResult(400, { code: 'VALIDATION_FAILED' }), null);
  assert.equal(toVerifyResult(404, null), null); // 서버가 아닌 곳의 404
  assert.equal(toVerifyResult(200, null), null); // 프록시 미설정으로 HTML이 온 경우
  assert.equal(toVerifyResult(201, { result: 'VERIFIED', pickup: null }), null);
  assert.equal(toVerifyResult(201, { result: 'REJECTED', reasonCode: 'QR_REVOKED' }), null); // QR 확인은 아직 연결 전
});

test('요청 ID는 서버 형식에 맞고 매번 다르다', () => {
  const a = newVerificationRequestId();
  assert.match(a, /^[A-Za-z0-9_.-]{1,64}$/);
  assert.notEqual(a, newVerificationRequestId());
});
