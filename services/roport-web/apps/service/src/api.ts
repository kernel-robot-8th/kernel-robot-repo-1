// 서버 계약(서비스 서버·고객 웹 첫 슬라이스)의 응답 타입. 필드를 추가하지 않는다.
export type Member = { memberId: string; displayName: string };
export type ActivePickup = { pickupId: string; reservationNo: string; status: string };
export type Order = {
  orderId: string;
  orderNo: string;
  orderedAt: string;
  businessStatus: string;
  items: { itemId: string; productName: string; quantity: number }[];
  activePickup: ActivePickup | null;
};
export type Location = { locationId: string; name: string };
export type Flight = {
  flightId: string;
  flightNo: string;
  serviceDate: string;
  boardingStart: string;
  boardingEnd: string;
  departureAt: string;
  gateCode: string;
};
export type Pickup = {
  pickupId: string;
  reservationNo: string;
  orderId: string;
  orderNo: string;
  location: Location;
  flight: Flight & { revision: number };
  pickupAt: string;
  status: string;
  revision: number;
  createdAt: string;
};
export type PickupRequest = {
  requestId: string;
  orderId: string;
  locationId: string;
  flightId: string;
  pickupAt: string;
};

export class ApiError extends Error {
  // 파라미터 프로퍼티 대신 명시 필드: node --test type stripping이 지원하는 문법만 쓴다.
  readonly status: number;
  readonly code: string | null;
  constructor(status: number, code: string | null, message: string) {
    super(message);
    this.status = status;
    this.code = code;
  }
}

async function call<T>(method: string, path: string, body?: unknown): Promise<T> {
  let res: Response;
  try {
    res = await fetch(path, {
      method,
      headers: body === undefined ? undefined : { 'Content-Type': 'application/json' },
      body: body === undefined ? undefined : JSON.stringify(body),
    });
  } catch {
    throw new ApiError(0, 'NETWORK', '서버에 연결하지 못했습니다.');
  }
  if (!res.ok) {
    let code: string | null = null;
    try {
      code = ((await res.json()) as { code?: string }).code ?? null;
    } catch {
      /* problem+json이 아닌 응답 */
    }
    throw new ApiError(res.status, code, `HTTP ${res.status}`);
  }
  if (res.status === 204) return undefined as T;
  try {
    return (await res.json()) as T;
  } catch {
    // 200인데 JSON이 아님(예: 프록시 오설정으로 index.html 반환)
    throw new ApiError(res.status, 'BAD_RESPONSE', '서버 응답을 해석하지 못했습니다.');
  }
}

export const api = {
  me: () => call<Member>('GET', '/api/session'),
  login: (loginId: string) => call<Member>('POST', '/api/session', { loginId }),
  logout: () => call<void>('DELETE', '/api/session'),
  orders: () => call<Order[]>('GET', '/api/orders'),
  locations: () => call<Location[]>('GET', '/api/pickup-locations'),
  flights: () => call<Flight[]>('GET', '/api/flights'),
  times: (orderId: string, flightId: string, locationId: string) =>
    call<{ times: string[] }>(
      'GET',
      `/api/orders/${encodeURIComponent(orderId)}/pickup-times?flightId=${encodeURIComponent(flightId)}&locationId=${encodeURIComponent(locationId)}`,
    ),
  reserve: (r: PickupRequest) => call<Pickup>('POST', '/api/pickups', r),
  pickups: () => call<Pickup[]>('GET', '/api/pickups'),
};

// 계약상 쓰는 상태는 REQUESTED뿐. 도착·직원 확인·인도 완료를 뜻하지 않는다.
export function statusLabel(status: string): string {
  return status === 'REQUESTED' ? '예약 접수됨 – 현장 배정 전' : `알 수 없는 상태 (${status})`;
}

const BAD_RESPONSE_TEXT = '서버 응답 형식이 올바르지 않습니다. 서버 주소·프록시 설정을 확인해 주세요.';

const RESERVE_ERRORS: Record<string, string> = {
  VALIDATION_FAILED: '입력값이 올바르지 않습니다. 선택 내용을 확인해 주세요.',
  ORDER_NOT_FOUND: '주문을 찾을 수 없습니다.',
  LOCATION_NOT_FOUND: '수령 위치를 찾을 수 없습니다.',
  FLIGHT_NOT_FOUND: '항공편을 찾을 수 없습니다.',
  ORDER_NOT_RESERVABLE: '이 주문은 현재 예약할 수 없습니다.',
  ACTIVE_PICKUP_EXISTS: '이미 진행 중인 예약이 있는 주문입니다.',
  IDEMPOTENCY_CONFLICT: '같은 요청 번호로 다른 내용이 접수되어 있습니다. 새 요청 번호를 발급했으니 내용을 확인하고 다시 예약해 주세요.',
  PICKUP_TIME_NOT_AVAILABLE: '선택한 시간은 더 이상 예약할 수 없습니다. 시간을 다시 선택해 주세요.',
  NETWORK: '서버에 연결하지 못했습니다. 같은 요청으로 다시 시도할 수 있습니다.',
  BAD_RESPONSE: BAD_RESPONSE_TEXT,
};

export function reserveErrorText(e: unknown): string {
  if (e instanceof ApiError) {
    return (e.code && RESERVE_ERRORS[e.code]) || `예약 결과를 확인하지 못했습니다 (${e.code ?? e.status}). 같은 요청으로 다시 시도하거나 예약 조회에서 확인해 주세요.`;
  }
  return '알 수 없는 오류가 발생했습니다.';
}

// 표시는 한국 시간(KST), 서버 전송은 서버가 준 UTC ISO 문자열 그대로.
export const fmt = (iso: string) =>
  new Date(iso).toLocaleString('ko-KR', { timeZone: 'Asia/Seoul', dateStyle: 'short', timeStyle: 'short' }) + ' KST';

export function loadErrorText(e: unknown): string {
  const code = e instanceof ApiError ? e.code : null;
  if (code === 'NETWORK') return '서버에 연결하지 못했습니다.';
  if (code === 'BAD_RESPONSE') return BAD_RESPONSE_TEXT;
  return '불러오지 못했습니다. 잠시 후 다시 시도해 주세요.';
}

// 시간 목록 조회 오류: 예약과 같은 code 문구가 있으면 그것을, 없으면 불러오기 실패 문구.
export const timesErrorText = (e: unknown) =>
  e instanceof ApiError && e.code && e.code in RESERVE_ERRORS ? reserveErrorText(e) : loadErrorText(e);

// 선택한 시간이 사라졌다는 오류 뒤에는 시간 목록을 다시 조회해야 같은 422가 반복되지 않는다.
export const shouldReloadTimes = (e: unknown) => e instanceof ApiError && e.code === 'PICKUP_TIME_NOT_AVAILABLE';

// 탑승 시작이 이미 지난 항공편은 숨긴다(같은 시각은 유지).
export const upcomingFlights = (flights: Flight[], now: Date = new Date()) =>
  flights.filter((f) => new Date(f.boardingStart).getTime() >= now.getTime());

// crypto.randomUUID는 secure context(HTTPS·localhost)에서만 있어 LAN IP HTTP에서는 getRandomValues로 v4 UUID를 만든다.
export function newRequestId(c: Crypto = globalThis.crypto): string {
  const b = c.getRandomValues(new Uint8Array(16));
  b[6] = (b[6] & 0x0f) | 0x40;
  b[8] = (b[8] & 0x3f) | 0x80;
  const h = Array.from(b, (x) => x.toString(16).padStart(2, '0')).join('');
  return `${h.slice(0, 8)}-${h.slice(8, 12)}-${h.slice(12, 16)}-${h.slice(16, 20)}-${h.slice(20)}`;
}

export function loginErrorText(e: unknown): string {
  const code = e instanceof ApiError ? e.code : null;
  if (code === 'LOGIN_FAILED') return '로그인할 수 없습니다. 로그인 ID를 확인해 주세요.';
  if (code === 'VALIDATION_FAILED') return '로그인 ID를 입력해 주세요.';
  return loadErrorText(e);
}

// 선택이 바뀌면 새 요청이므로 새 requestId. 같은 선택의 재시도(네트워크·결과 불명 오류)는 유지.
// 같은 번호로는 같은 충돌이 반복되므로 IDEMPOTENCY_CONFLICT만 새로 발급.
export function nextRequestId(
  current: string,
  event: { type: 'selection' } | { type: 'error'; error: unknown },
  fresh: () => string,
): string {
  if (event.type === 'selection') return fresh();
  return event.error instanceof ApiError && event.error.code === 'IDEMPOTENCY_CONFLICT' ? fresh() : current;
}
