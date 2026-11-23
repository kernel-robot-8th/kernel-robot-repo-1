// 화면에서 쓰는 타입. 필드 이름은 서비스 서버 첫 슬라이스(FFT1-166) 응답을 따른다.
// VerifiedPickup은 카트 예약 확인 API(FFT1-178) 응답의 pickup과 같다.
// 수령 확인은 서비스 API를 사용한다. 안내·추종은 데모에만 제공한다.

export type Location = { locationId: string; name: string };

export type Flight = {
  flightNo: string;
  boardingStart: string; // ISO 8601 (UTC)
  boardingEnd: string;
  gateCode: string; // 예: G1
};

export type OrderItem = { itemId: string; productName: string; quantity: number };

// 인도 승인된 예약. 서비스 서버가 QR·예약번호를 확인한 결과로만 만든다.
export interface VerifiedPickup {
  verificationId?: string; // 서버 확인 결과. 데모에는 없음.
  pickupId: string;
  reservationNo: string;
  orderNo: string;
  customerName: string;
  location: Location;
  flight: Flight;
  items: OrderItem[];
  packageCount: number;
}

// 입국장 수령 후 선택하는 서비스(SR-21 게이트 안내 P0, SR-22 짐 운반·추종 P1).
export type ServiceMode = 'GATE_GUIDE' | 'FOLLOW';

// NOT_ACTIVE: 취소됐거나 주문이 확정되지 않아 수령할 수 없는 예약(서버 PICKUP_NOT_ACTIVE).
// TOO_MANY_ATTEMPTS: 이 카트의 확인 실패가 많아 서버가 잠시 막음(429, FFT1-182). retryAfterS는 서버가 알려 준 남은 초.
export type VerifyFailure = 'NOT_FOUND' | 'WRONG_CART' | 'ALREADY_RECEIVED' | 'NOT_ACTIVE' | 'TOO_MANY_ATTEMPTS';

export type VerifyResult = { ok: true; pickup: VerifiedPickup } | { ok: false; reason: VerifyFailure; retryAfterS?: number };
