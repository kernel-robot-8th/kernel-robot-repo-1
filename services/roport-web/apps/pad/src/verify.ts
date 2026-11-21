// 서비스 서버 카트 예약 확인(POST /cart-api/verifications, FFT1-178) 응답을 화면 결과로 바꾼다.
// fetch와 분리한 순수 함수라 node --test로 검사한다.
import type { VerifiedPickup, VerifyResult } from './types';

// 서버 계약: verifications 요청 ID는 [A-Za-z0-9_.-]{1,64}.
// crypto.randomUUID는 https·localhost에서만 있어 LAN http 접속에서도 되는 getRandomValues를 쓴다.
export function newVerificationRequestId(): string {
  const bytes = crypto.getRandomValues(new Uint8Array(16));
  return `pad-${Array.from(bytes, (b) => b.toString(16).padStart(2, '0')).join('')}`;
}

type Verification = {
  result?: string;
  reasonCode?: string | null;
  pickup?: VerifiedPickup | null;
};

// 반환 null은 "통신·설정 오류"(화면: 서버와 통신하지 못했습니다)다.
// 예약 없음(404)·확인 거절·시도 제한(429)만 고객에게 사유를 보여 준다. 401(카트 인증 꺼짐)·409(요청 ID 충돌)·400은 고객이 고칠 수 없어 오류로 둔다.
// 서버는 아직 카트 배정을 대조하지 않아(cartAssignmentChecked=false) WRONG_CART는 나오지 않는다.
export function toVerifyResult(status: number, body: unknown, retryAfter: string | null = null, now: number = Date.now()): VerifyResult | null {
  if (status === 404 && code(body) === 'RESERVATION_NOT_FOUND') return { ok: false, reason: 'NOT_FOUND' };
  if (status === 429 && code(body) === 'TOO_MANY_ATTEMPTS') {
    const seconds = retryAfterSeconds(retryAfter, now);
    return { ok: false, reason: 'TOO_MANY_ATTEMPTS', ...(seconds !== undefined && { retryAfterS: seconds }) };
  }
  if (status !== 200 && status !== 201) return null;
  const v = (body ?? {}) as Verification;
  if (v.result === 'VERIFIED') return isPickup(v.pickup) ? { ok: true, pickup: v.pickup } : null;
  if (v.result !== 'REJECTED') return null;
  if (v.reasonCode === 'ALREADY_RECEIVED') return { ok: false, reason: 'ALREADY_RECEIVED' };
  if (v.reasonCode === 'PICKUP_NOT_ACTIVE') return { ok: false, reason: 'NOT_ACTIVE' };
  return null;
}

// 화면이 바로 쓰는 필드만 확인한다. 빠지면 물품 화면이 깨지므로 통신 오류로 둔다.
function isPickup(p: unknown): p is VerifiedPickup {
  if (!p || typeof p !== 'object') return false;
  const v = p as Partial<VerifiedPickup>;
  return (
    typeof v.pickupId === 'string' &&
    typeof v.customerName === 'string' &&
    Array.isArray(v.items) &&
    typeof v.flight?.gateCode === 'string' &&
    typeof v.location?.locationId === 'string'
  );
}

// 서비스 서버는 초 단위 정수를 보내지만, 프록시가 HTTP 날짜 형식으로 바꿔도 해석한다.
// 지나치게 큰 값은 버리지 않고 1시간으로 제한하고, 해석할 수 없거나 이미 지난 시각이면 시간 없이 안내한다.
const MAX_RETRY_AFTER_S = 3600;
function retryAfterSeconds(value: string | null, now: number): number | undefined {
  const v = value?.trim();
  if (!v) return undefined;
  let seconds: number;
  if (/^\d+$/.test(v)) {
    seconds = Number(v);
  } else {
    const at = Date.parse(v);
    if (Number.isNaN(at)) return undefined;
    seconds = Math.ceil((at - now) / 1000);
  }
  return seconds >= 1 ? Math.min(seconds, MAX_RETRY_AFTER_S) : undefined;
}

function code(body: unknown): string | null {
  return body && typeof body === 'object' && 'code' in body && typeof body.code === 'string' ? body.code : null;
}
