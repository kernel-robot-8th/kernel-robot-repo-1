// 화면에서 쓰는 타입. 필드 이름은 서비스 서버 첫 슬라이스(FFT1-166) 응답을 따른다.
// QR 인증·수령 확인·서비스 선택 API는 아직 없어 아래 형태는 제안이며 확정 규격이 아니다.

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

export type VerifyFailure = 'NOT_FOUND' | 'WRONG_CART' | 'ALREADY_RECEIVED';

export type VerifyResult = { ok: true; pickup: VerifiedPickup } | { ok: false; reason: VerifyFailure };
