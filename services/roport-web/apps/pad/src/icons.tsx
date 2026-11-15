// 화면용 선 아이콘. 외부 아이콘 라이브러리 없이 그린다.

const PATHS = {
  arrow: 'M5 12h14M13 6l6 6-6 6',
  check: 'M5 12.5l4.5 4.5L19 7.5',
  bag: 'M5 8h14l-1.2 12.2a1 1 0 0 1-1 .8H7.2a1 1 0 0 1-1-.8L5 8zM9 8V6.5a3 3 0 0 1 6 0V8',
  plane: 'M10.5 21l2-1.5V14l8 2.5V14.5l-8-5V4a1.5 1.5 0 0 0-3 0v5.5l-8 5V16.5l8-2.5v5.5l-2 1.5',
  gate: 'M4 21V5a2 2 0 0 1 2-2h12a2 2 0 0 1 2 2v16M2 21h20M9 12h6M13 9l3 3-3 3',
  follow: 'M8 4.5a1.8 1.8 0 1 0 0 .01M8 8v5l-2.5 7M8 13l2.5 7M5 10.5h6M15 12h5l-1 6h-4l-1-6zM16 18v2M19 18v2',
  pause: 'M9 5v14M15 5v14',
  play: 'M7 5l12 7-12 7V5z',
  alert: 'M12 3l10 18H2L12 3zM12 10v5M12 18h.01',
  keyboard: 'M3 6h18v12H3zM7 10h.01M11 10h.01M15 10h.01M7 14h10',
} as const;

export type IconName = keyof typeof PATHS;

export function Icon({ name }: { name: IconName }) {
  return (
    <svg className="icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d={PATHS[name]} />
    </svg>
  );
}

// 대기 화면 그림: 휴대폰 화면의 QR을 카메라에 비추는 모습. 실제 QR 데이터가 아니다.
export function PhoneQr() {
  const cells = [0, 1, 2, 4, 6, 7, 8, 9, 12, 14, 16, 18, 20, 22, 25, 27, 30, 31, 33, 35, 36, 38, 40, 44, 46, 48];
  return (
    <svg className="phone-qr" viewBox="0 0 160 240" aria-hidden="true">
      <rect className="phone-body" x="20" y="10" width="120" height="220" rx="16" />
      <rect className="phone-screen" x="30" y="34" width="100" height="172" rx="4" />
      <rect className="phone-notch" x="62" y="20" width="36" height="5" rx="2.5" />
      <g transform="translate(45 75) scale(1)">
        <rect className="qr" x="0" y="0" width="20" height="20" rx="2" /><rect className="qr" x="50" y="0" width="20" height="20" rx="2" /><rect className="qr" x="0" y="50" width="20" height="20" rx="2" />
        <rect className="qr-hole" x="5" y="5" width="10" height="10" /><rect className="qr-hole" x="55" y="5" width="10" height="10" /><rect className="qr-hole" x="5" y="55" width="10" height="10" />
        {cells.map((i) => <rect key={i} className="qr" x={26 + (i % 7) * 6} y={26 + Math.floor(i / 7) * 6} width="5" height="5" />)}
      </g>
      <text className="phone-caption" x="80" y="180" textAnchor="middle">예약 QR</text>
    </svg>
  );
}
