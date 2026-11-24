// 패드가 서버와 주고받는 모든 호출을 이 파일에 모은다.
// 기본은 데모. VITE_PAD_API=server면 QR/예약번호 확인과 수령 저장을 서비스 서버에 요청한다.
// 데모 응답은 서버·로봇 동작을 확인한 결과가 아니다.
import type { Location, ServiceMode, VerifiedPickup, VerifyResult } from './types';
import { newVerificationRequestId, toVerifyResult } from './verify';
import { boardingAlertDelayMs, framePath, parseFollowStatus, parseReceiptIntent, receiptIntent, type FollowAction, type FollowMode, type FollowStatus, type ReceiptIntent } from './follow';

// 배포 nginx에는 아직 /cart-api 전달이 없어 기본을 데모로 둔다.
const USE_SERVER = import.meta.env.VITE_PAD_API === 'server';

export const FOLLOW_AVAILABLE = USE_SERVER && import.meta.env.VITE_PAD_FOLLOW === '1';

export const IS_DEMO = !USE_SERVER;

// 데모에서만 수령 저장이 가상 응답이다.
export const RECEIPT_IS_DEMO = !USE_SERVER;

// 이 패드가 붙은 카트(서버 X-Cart-Id, 개발 전용 카트 인증). 비밀값이 아니다.
export const CART_ID = import.meta.env.VITE_PAD_CART_ID || 'pinky3';
export const CART_LABEL = `스마트카트 · ${CART_ID}`;

// 서비스 서버의 예약번호 문자 집합(0·O·1·I 제외, 8자리).
export const RESERVATION_ALPHABET = '23456789ABCDEFGHJKLMNPQRSTUVWXYZ';
export const RESERVATION_LENGTH = 8;

// 서버 계약에 위치 구분 필드가 없어 개발 시드의 입국장 ID로 판별한다. 계약이 정해지면 교체한다.
export const isArrivalHall = (location: Location) => location.locationId === 'dev-loc-arrival';

export const gateLabel = (gateCode: string) => gateCode.replace(/^G(\d+)$/, 'Gate $1');

function hoursFromNow(hours: number): string {
  return new Date(Date.now() + hours * 3_600_000).toISOString();
}

// 가짜 데이터만 둔다. 이름·주문·상품은 서비스 서버 개발 시드(db/dev/seed.sql)에 맞췄다.
const DEMO_PICKUPS: Array<VerifiedPickup & { cartId: string; received: boolean }> = [
  {
    pickupId: 'demo-pickup-1',
    reservationNo: 'GT2X7KQA',
    orderNo: 'DEV-0001',
    customerName: '테스트 고객 1',
    location: { locationId: 'dev-loc-gate2', name: 'Gate 2' },
    flight: { flightNo: 'ZZ202', gateCode: 'G2', boardingStart: hoursFromNow(0.7), boardingEnd: hoursFromNow(1.2) },
    items: [
      { itemId: 'dev-item-1', productName: '샘플 향수 A', quantity: 1 },
      { itemId: 'dev-item-2', productName: '샘플 화장품 B', quantity: 2 },
    ],
    packageCount: 1,
    cartId: 'pinky3',
    received: false,
  },
  {
    pickupId: 'demo-pickup-2',
    reservationNo: 'ARV9H3MP',
    orderNo: 'DEV-0002',
    customerName: '테스트 고객 1',
    location: { locationId: 'dev-loc-arrival', name: '입국장' },
    flight: { flightNo: 'ZZ303', gateCode: 'G3', boardingStart: hoursFromNow(0.4), boardingEnd: hoursFromNow(0.9) },
    items: [{ itemId: 'dev-item-3', productName: '샘플 초콜릿 C', quantity: 3 }],
    packageCount: 1,
    cartId: 'pinky3',
    received: false,
  },
  {
    pickupId: 'demo-pickup-3',
    reservationNo: 'DNE5W8RC',
    orderNo: 'DEV-0003',
    customerName: '테스트 고객 1',
    location: { locationId: 'dev-loc-gate1', name: 'Gate 1' },
    flight: { flightNo: 'ZZ101', gateCode: 'G1', boardingStart: hoursFromNow(1.5), boardingEnd: hoursFromNow(2) },
    items: [{ itemId: 'dev-item-4', productName: '샘플 시계 D', quantity: 1 }],
    packageCount: 1,
    cartId: 'pinky3',
    received: true,
  },
];

// 데모 조작 버튼에 쓰는 예약(설명, 예약번호).
export const DEMO_QR: Array<[string, string]> = [
  ['Gate 수령', 'GT2X7KQA'],
  ['입국장 수령', 'ARV9H3MP'],
  ['수령 완료된 예약', 'DNE5W8RC'],
];

const delay = (ms: number) => new Promise((resolve) => setTimeout(resolve, ms));

function verify(reservationNo: string): VerifyResult {
  const found = DEMO_PICKUPS.find((p) => p.reservationNo === reservationNo);
  if (!found) return { ok: false, reason: 'NOT_FOUND' };
  if (found.cartId !== CART_ID) return { ok: false, reason: 'WRONG_CART' };
  if (found.received) return { ok: false, reason: 'ALREADY_RECEIVED' };
  const { cartId: _cartId, received: _received, ...pickup } = found;
  return { ok: true, pickup };
}

// 카트 카메라가 읽은 예약 QR을 서비스 서버에 확인 요청한다(SR-18-1·18-2).
export async function verifyQr(qrPayload: string): Promise<VerifyResult> {
  if (USE_SERVER) return verifyServer('QR', qrPayload);
  await delay(600);
  return verify(qrPayload);
}

// QR 인식 실패 시 예약번호로 확인한다(SR-18-2-E).
// 서버 모드: 통신·설정 오류면 throw하고 화면은 "서버와 통신하지 못했습니다"를 보여 준다.
export async function verifyReservationNo(reservationNo: string): Promise<VerifyResult> {
  const value = reservationNo.trim().toUpperCase();
  if (!USE_SERVER) {
    await delay(600);
    return verify(value);
  }
  return verifyServer('RESERVATION_NO', value);
}

async function verifyServer(method: 'QR' | 'RESERVATION_NO', value: string): Promise<VerifyResult> {
  const body = JSON.stringify({ verificationRequestId: newVerificationRequestId(), method, value });
  const result = toVerifyResult(...(await postCart('/cart-api/verifications', body)));
  if (!result) throw new Error('예약 확인 실패');
  return result;
}

// 응답을 못 받았을 때(연결 실패·8초 초과)만 같은 요청 ID로 한 번 더 보낸다. 서버는 같은 ID를 한 번만 기록한다.
async function postCart(path: string, body: string): Promise<[number, unknown, string | null]> {
  const send = () =>
    fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', 'X-Cart-Id': CART_ID },
      body,
      signal: AbortSignal.timeout(8000),
    });
  let res: Response;
  try {
    res = await send();
  } catch {
    res = await send();
  }
  let json: unknown = null;
  try {
    json = await res.json();
  } catch {
    // JSON이 아닌 응답(예: 프록시 미설정으로 index.html) — toVerifyResult가 오류로 처리한다.
  }
  return [res.status, json, res.headers.get('Retry-After')];
}

// 고객의 수령 확인. 서비스 서버가 수령 완료를 기록한다(SR-18-2).
// 서버가 배정·도착·확인 유효성을 검사한 뒤에만 완료를 표시한다.
export interface ReceiptResult { accepted: boolean; receiptId?: string }
const RECEIPT_KEY = `roport.pad.receipt.${CART_ID}`;

// ---- 탑승 시간 알림(패드가 판단, 기본 끔) ----
// VITE_PAD_BOARDING_ALERT_S: 실제 추종이 확인된 뒤 알림까지의 초(시연·시험 20). 없으면 알림을 띄우지 않는다.
export const BOARDING_ALERT_DELAY_MS = boardingAlertDelayMs(import.meta.env.VITE_PAD_BOARDING_ALERT_S);
// VITE_PAD_BOARDING_ROBOT=1: 같은 PC 의 인식 도구(roport_follow tool --boarding-alert)에 로봇 표시(LCD·LED·부저)를 부탁한다.
// 개발 서버 프록시(/follow-tool)로만 간다. 주행 명령이 아니고, 실패해도 패드 창은 그대로 둔다.
const BOARDING_ROBOT = import.meta.env.VITE_PAD_BOARDING_ROBOT === '1';
const BOARDING_SHOWN_KEY = `roport.pad.boarding-alert.${CART_ID}`;

// 한 수령(receipt)에 한 번만 알린다. 새로고침해도 다시 띄우지 않게 수령 ID 만 저장한다(예약·고객 정보 없음).
export function boardingAlertShown(receiptId: string): boolean {
  try { return sessionStorage.getItem(BOARDING_SHOWN_KEY) === receiptId; } catch { return false; }
}
export function markBoardingAlertShown(receiptId: string): void {
  try { sessionStorage.setItem(BOARDING_SHOWN_KEY, receiptId); } catch { /* 저장 못 해도 창은 띄운다 */ }
}
export async function robotBoardingAlert(on: boolean): Promise<void> {
  if (!BOARDING_ROBOT) return;
  try {
    await fetch('/follow-tool/api/boarding_alert', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ on }),
      keepalive: true, signal: AbortSignal.timeout(3000),
    });
  } catch { /* 로봇 표시는 보조 수단 */ }
}
const RECEIPT_PENDING_KEY = `roport.pad.receipt-pending.${CART_ID}`;

export function savedReceipt(): string | null {
  if (!USE_SERVER) return null;
  try { return sessionStorage.getItem(RECEIPT_KEY); } catch { return null; }
}
export function pendingReceipt(): ReceiptIntent | null {
  if (!USE_SERVER) return null;
  try { return parseReceiptIntent(JSON.parse(sessionStorage.getItem(RECEIPT_PENDING_KEY) || 'null')); } catch { return null; }
}
export function forgetReceipt(): void { sessionStorage.removeItem(RECEIPT_KEY); }

export async function confirmReceipt(pickup: VerifiedPickup, mode: FollowMode = 'none', card?: number): Promise<ReceiptResult> {
  if (USE_SERVER) {
    if (!pickup.verificationId) return { accepted: false };
    const intent = receiptIntent(pickup.verificationId, mode, card);
    const pending = pendingReceipt();
    if (pending && JSON.stringify(pending) !== JSON.stringify(intent)) {
      throw new Error('이전 수령 확인 결과를 먼저 확인해 주세요.');
    }
    // Persist only the immutable request, never reservation/customer data or camera images.
    sessionStorage.setItem(RECEIPT_PENDING_KEY, JSON.stringify(intent));
    return retryReceipt();
  }
  await delay(500);
  const found = DEMO_PICKUPS.find((p) => p.pickupId === pickup.pickupId);
  if (found) found.received = true;
  return { accepted: Boolean(found) };
}

// User initiated only: reloading never repeats a receipt or robot command.
export async function retryReceipt(): Promise<ReceiptResult> {
  const intent = pendingReceipt();
  if (!intent) throw new Error('수령 확인 기록이 없습니다.');
  const [status, body] = await postCart('/cart-api/receipts', JSON.stringify(intent));
  if ((status === 200 || status === 201) && body && typeof body === 'object' && 'receiptId' in body && typeof body.receiptId === 'string' && body.receiptId) {
    sessionStorage.setItem(RECEIPT_KEY, body.receiptId);
    sessionStorage.removeItem(RECEIPT_PENDING_KEY);
    return { accepted: true, receiptId: body.receiptId };
  }
  if (status >= 400 && status < 500 && body && typeof body === 'object') {
    sessionStorage.removeItem(RECEIPT_PENDING_KEY);
    return { accepted: false };
  }
  throw new Error('수령 확인 결과를 받지 못했습니다. 같은 내용으로 다시 확인해 주세요.');
}

async function getCart(path: string, signal?: AbortSignal): Promise<Response> {
  return fetch(path, { headers: { 'X-Cart-Id': CART_ID }, cache: 'no-store', signal: signal ?? AbortSignal.timeout(8000) });
}

export async function followCapabilities(verificationId: string): Promise<FollowMode[]> {
  const res = await getCart(`/cart-api/follow/capabilities?verificationId=${encodeURIComponent(verificationId)}`);
  if (!res.ok) throw new Error('추종 지원 상태를 확인하지 못했습니다.');
  const data: unknown = await res.json();
  if (!data || typeof data !== 'object' || !('modes' in data) || !Array.isArray(data.modes) ||
      data.modes.some(m => m !== 'marker' && m !== 'appearance')) throw new Error('추종 지원 상태를 확인하지 못했습니다.');
  return data.modes;
}

export async function followStatus(receiptId: string, signal?: AbortSignal): Promise<FollowStatus> {
  const res = await getCart(`/cart-api/follow/${encodeURIComponent(receiptId)}`, signal);
  if (!res.ok) throw new Error('카트 상태를 확인하지 못했습니다. 안내 직원에게 문의해 주세요.');
  const status = parseFollowStatus(await res.json(), receiptId);
  if (!status) throw new Error('카트 상태 응답을 확인하지 못했습니다.');
  return status;
}

export async function followAction(receiptId: string, action: FollowAction): Promise<boolean> {
  const [status, body] = await postCart(`/cart-api/follow/${encodeURIComponent(receiptId)}/actions`, JSON.stringify(action));
  if (status >= 200 && status < 300 && body && typeof body === 'object' && 'accepted' in body && body.accepted === true) return true;
  if (status >= 400 && status < 500) return false;
  throw new Error('요청 결과를 확인하지 못했습니다.');
}

export async function followImage(receiptId: string, frameRef: string, path: string, signal: AbortSignal): Promise<Blob> {
  if (path !== framePath(receiptId, frameRef)) throw new Error('사진 경로를 확인하지 못했습니다.');
  const res = await getCart(path, signal);
  const contentType = res.headers.get('content-type')?.split(';')[0];
  if (!res.ok || !['image/jpeg', 'image/png'].includes(contentType || '')) throw new Error('사진을 불러오지 못했습니다. 다시 찍어 주세요.');
  return res.blob();
}

// 아래 요청은 실제 연동 시 관제를 거쳐 로봇 명령이 된다. 응답은 "접수"일 뿐 로봇 동작 완료가 아니다.
export async function requestService(pickupId: string, mode: ServiceMode): Promise<{ accepted: boolean }> {
  if (USE_SERVER) return { accepted: false };
  await delay(500);
  return { accepted: Boolean(pickupId && mode) };
}

export async function requestPause(pickupId: string): Promise<{ accepted: boolean }> {
  if (USE_SERVER) return { accepted: false };
  await delay(300);
  return { accepted: Boolean(pickupId) };
}

export async function requestResume(pickupId: string): Promise<{ accepted: boolean }> {
  if (USE_SERVER) return { accepted: false };
  await delay(300);
  return { accepted: Boolean(pickupId) };
}

// 데모 복구용: 수령 완료 표시를 처음 상태로 되돌린다.
export function resetDemo(): void {
  DEMO_PICKUPS.forEach((p) => {
    p.received = p.pickupId === 'demo-pickup-3';
  });
}

export interface ShoppingStatus {
  ownsCamera: boolean;
  stopped: boolean;
  simulation: boolean;
  canStart: boolean;
  notice: 'INACTIVE' | 'SHOPPING' | 'COME_CLOSER' | 'BLOCKED' | 'UNAVAILABLE' | 'PAUSED';
  selected: boolean;
  armed: boolean;
  state: string;
  drive: 'HOLD' | 'APPROACH';
  reason: string;
  frame: string | null;
  detecting: boolean;
  boxes: number[][];
  box: number[] | null;
}

export async function shopping(session: string, action = 'status', args: Record<string, unknown> = {}): Promise<ShoppingStatus> {
  const res = await fetch('/shopping/' + action, action === 'status'
    ? {headers: {'X-Shopping-Session': session}, cache: 'no-store', signal: AbortSignal.timeout(1500)}
    : {method: 'POST', headers: {'Content-Type':'application/json'}, signal: AbortSignal.timeout(2000),
       body: JSON.stringify({session, ...args})});
  // A restarted cart has forgotten this session. Read the camera status again.
  if (action === 'heartbeat' && res.status === 409) return shopping(session);
  if (!res.headers.get('content-type')?.includes('application/json')) throw new Error('카트 카메라 연결을 기다리고 있어요.');
  const data = await res.json();
  if (!res.ok) throw new Error(data.error || '카트 연결을 확인해 주세요.');
  return data;
}

export async function shoppingImage(session: string, path: string, signal: AbortSignal): Promise<Blob | null> {
  const res = await fetch(path, {headers: {'X-Shopping-Session': session}, cache: 'no-store', signal});
  return res.ok && res.headers.get('content-type') === 'image/jpeg' ? res.blob() : null;
}
