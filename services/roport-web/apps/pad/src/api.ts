// 패드가 서버와 주고받는 모든 호출을 이 파일에 모은다.
// 기본은 데모 데이터로 응답한다. 빌드 때 VITE_PAD_API=server면 예약번호 확인만 서비스 서버에 묻는다.
// 데모 응답은 서버·로봇 동작을 확인한 결과가 아니다.
import type { Location, ServiceMode, VerifiedPickup, VerifyResult } from './types';
import { newVerificationRequestId, toVerifyResult } from './verify';

// 배포 nginx에는 아직 /cart-api 전달이 없어 기본을 데모로 둔다.
const USE_SERVER = import.meta.env.VITE_PAD_API === 'server';

// 서버 모드에서도 QR 확인·수령 확인·서비스 요청은 아직 데모 응답이라 데모 표시를 유지한다.
export const IS_DEMO = true;

// 서버에서 확인한 예약의 수령 완료는 저장되지 않는다. 완료 화면에 그 사실을 적는다.
export const RECEIPT_IS_DEMO = USE_SERVER;

// 이 패드가 붙은 카트(서버 X-Cart-Id, 개발 전용 카트 인증). 비밀값이 아니다.
export const CART_ID = import.meta.env.VITE_PAD_CART_ID || 'pinky3';
export const CART_LABEL = '스마트카트 3호기';

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
  const body = JSON.stringify({ verificationRequestId: newVerificationRequestId(), method: 'RESERVATION_NO', value });
  const result = toVerifyResult(...(await postVerification(body)));
  if (!result) throw new Error('예약 확인 실패');
  return result;
}

// 응답을 못 받았을 때(연결 실패·8초 초과)만 같은 요청 ID로 한 번 더 보낸다. 서버는 같은 ID를 한 번만 기록한다.
async function postVerification(body: string): Promise<[number, unknown]> {
  const send = () =>
    fetch('/cart-api/verifications', {
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
  return [res.status, json];
}

// 고객의 수령 확인. 서비스 서버가 수령 완료를 기록한다(SR-18-2).
// 수령 기록 API가 아직 없어 서버 모드에서도 데모로 접수한다(서버 예약은 수령 처리되지 않는다).
export async function confirmReceipt(pickupId: string): Promise<{ accepted: boolean }> {
  await delay(500);
  const found = DEMO_PICKUPS.find((p) => p.pickupId === pickupId);
  if (found) found.received = true;
  return { accepted: Boolean(found) || (USE_SERVER && Boolean(pickupId)) };
}

// 아래 요청은 실제 연동 시 관제를 거쳐 로봇 명령이 된다. 응답은 "접수"일 뿐 로봇 동작 완료가 아니다.
export async function requestService(pickupId: string, mode: ServiceMode): Promise<{ accepted: boolean }> {
  await delay(500);
  return { accepted: Boolean(pickupId && mode) };
}

export async function requestPause(pickupId: string): Promise<{ accepted: boolean }> {
  await delay(300);
  return { accepted: Boolean(pickupId) };
}

export async function requestResume(pickupId: string): Promise<{ accepted: boolean }> {
  await delay(300);
  return { accepted: Boolean(pickupId) };
}

// 데모 복구용: 수령 완료 표시를 처음 상태로 되돌린다.
export function resetDemo(): void {
  DEMO_PICKUPS.forEach((p) => {
    p.received = p.pickupId === 'demo-pickup-3';
  });
}
