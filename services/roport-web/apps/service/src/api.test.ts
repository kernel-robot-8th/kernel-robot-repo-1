// 실행: services/roport-web/apps/service 에서 `node --test src/` (Node 24 type stripping, 의존성 없음)
import { test, afterEach } from 'node:test';
import assert from 'node:assert/strict';
import {
  api, ApiError, fmt, statusLabel, reserveErrorText, loginErrorText, loadErrorText, nextRequestId,
} from './api.ts';

const realFetch = globalThis.fetch;
afterEach(() => { globalThis.fetch = realFetch; });
const stub = (fn: () => Response | Promise<Response>) => { globalThis.fetch = (async () => fn()) as typeof fetch; };
const problem = (status: number, code?: string) =>
  new Response(JSON.stringify(code ? { code } : {}), { status, headers: { 'Content-Type': 'application/problem+json' } });

// 1. statusLabel
test('statusLabel: REQUESTED는 접수 문구', () => {
  assert.equal(statusLabel('REQUESTED'), '예약 접수됨 – 현장 배정 전');
});
test('statusLabel: 모르는 값은 원문과 함께 알 수 없음', () => {
  assert.equal(statusLabel('ARRIVED'), '알 수 없는 상태 (ARRIVED)');
  assert.equal(statusLabel('COMPLETED'), '알 수 없는 상태 (COMPLETED)');
});
test('statusLabel: 수령 완료/도착 문구 없음', () => {
  assert.doesNotMatch(statusLabel('REQUESTED'), /완료|도착|인도/);
});

// 2. 오류 문구
const RESERVE_CODES = [
  'VALIDATION_FAILED', 'ORDER_NOT_FOUND', 'LOCATION_NOT_FOUND', 'FLIGHT_NOT_FOUND',
  'ORDER_NOT_RESERVABLE', 'ACTIVE_PICKUP_EXISTS', 'IDEMPOTENCY_CONFLICT', 'PICKUP_TIME_NOT_AVAILABLE',
];
test('reserveErrorText: 예약 오류 8종은 각자 고유 문구', () => {
  const texts = RESERVE_CODES.map((c) => reserveErrorText(new ApiError(409, c, 'x')));
  for (const t of texts) assert.doesNotMatch(t, /확인하지 못했습니다/);
  assert.equal(new Set(texts).size, 8);
});
test('reserveErrorText: NETWORK는 같은 요청 재시도 안내', () => {
  assert.match(reserveErrorText(new ApiError(0, 'NETWORK', 'x')), /연결하지 못했습니다.*다시 시도/);
});
test('reserveErrorText: 모르는 code/status는 결과 불명 문구에 식별자 포함', () => {
  assert.match(reserveErrorText(new ApiError(500, 'BOOM', 'x')), /결과를 확인하지 못했습니다 \(BOOM\)/);
  assert.match(reserveErrorText(new ApiError(502, null, 'x')), /결과를 확인하지 못했습니다 \(502\)/);
  assert.equal(reserveErrorText(new Error('x')), '알 수 없는 오류가 발생했습니다.');
});
test('loginErrorText: LOGIN_FAILED, VALIDATION_FAILED, NETWORK, 기타', () => {
  assert.match(loginErrorText(new ApiError(401, 'LOGIN_FAILED', 'x')), /로그인할 수 없습니다/);
  assert.match(loginErrorText(new ApiError(400, 'VALIDATION_FAILED', 'x')), /로그인 ID를 입력/);
  assert.match(loginErrorText(new ApiError(0, 'NETWORK', 'x')), /연결하지 못했습니다/);
  assert.match(loginErrorText(new ApiError(500, 'BOOM', 'x')), /불러오지 못했습니다/);
});
test('loadErrorText: NETWORK와 그 외 구분', () => {
  assert.equal(loadErrorText(new ApiError(0, 'NETWORK', 'x')), '서버에 연결하지 못했습니다.');
  assert.match(loadErrorText(new ApiError(500, null, 'x')), /불러오지 못했습니다/);
  assert.match(loadErrorText(new Error('x')), /불러오지 못했습니다/);
});

// 3. requestId 정책
const fresh = () => 'NEW';
test('nextRequestId: 선택 변경은 새 값', () => {
  assert.equal(nextRequestId('OLD', { type: 'selection' }, fresh), 'NEW');
});
test('nextRequestId: 네트워크·결과 불명·그 외 오류는 유지(같은 선택 재시도)', () => {
  for (const e of [new ApiError(0, 'NETWORK', 'x'), new ApiError(500, null, 'x'), new ApiError(409, 'ACTIVE_PICKUP_EXISTS', 'x'), new Error('x')]) {
    assert.equal(nextRequestId('OLD', { type: 'error', error: e }, fresh), 'OLD');
  }
});
test('nextRequestId: IDEMPOTENCY_CONFLICT는 새 값', () => {
  assert.equal(nextRequestId('OLD', { type: 'error', error: new ApiError(409, 'IDEMPOTENCY_CONFLICT', 'x') }, fresh), 'NEW');
});

// 4. 시각
test('fmt: UTC ISO를 KST로 표시하고 KST 표기', () => {
  assert.equal(fmt('2026-01-01T15:30:00Z'), '26. 1. 2. 오전 12:30 KST'); // KST 2026-01-02 00:30
});

// 5. api 래퍼
test('call: 401은 ApiError(status 401, code)', async () => {
  stub(() => problem(401, 'UNAUTHENTICATED'));
  await assert.rejects(api.me(), (e: unknown) => e instanceof ApiError && e.status === 401 && e.code === 'UNAUTHENTICATED');
});
test('call: ProblemDetail code 파싱, 본문이 JSON 아니면 code null', async () => {
  stub(() => problem(409, 'ACTIVE_PICKUP_EXISTS'));
  await assert.rejects(api.pickups(), (e: unknown) => e instanceof ApiError && e.status === 409 && e.code === 'ACTIVE_PICKUP_EXISTS');
  stub(() => new Response('<html>', { status: 502 }));
  await assert.rejects(api.pickups(), (e: unknown) => e instanceof ApiError && e.status === 502 && e.code === null);
});
test('call: 204는 undefined', async () => {
  stub(() => new Response(null, { status: 204 }));
  assert.equal(await api.logout(), undefined);
});
test('call: 네트워크 오류는 ApiError(0, NETWORK)', async () => {
  stub(() => { throw new TypeError('fetch failed'); });
  await assert.rejects(api.orders(), (e: unknown) => e instanceof ApiError && e.status === 0 && e.code === 'NETWORK');
});
test('call: 성공 JSON 반환, body는 JSON으로 전송', async () => {
  let seen: { url: unknown; init?: RequestInit } | undefined;
  globalThis.fetch = (async (url: unknown, init?: RequestInit) => {
    seen = { url, init };
    return new Response(JSON.stringify({ memberId: 'm', displayName: 'd' }), { status: 200 });
  }) as typeof fetch;
  assert.deepEqual(await api.login('u1'), { memberId: 'm', displayName: 'd' });
  assert.equal(seen?.url, '/api/session');
  assert.equal(seen?.init?.method, 'POST');
  assert.equal(seen?.init?.body, '{"loginId":"u1"}');
});
