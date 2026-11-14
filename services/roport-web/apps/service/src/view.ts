// 화면 표시용 순수 함수. 서버 계약 필드만 읽고 새 상태를 만들어 내지 않는다.
import type { Flight, Location } from './api'; // 타입 전용 import라 node --test에서도 지워진다.

// 수령 위치 간략 지도: 이름이 "Gate N"인 위치는 게이트 줄에 번호순으로, 나머지(입국장 등)는 아래 줄에 둔다.
// 위치 ID를 하드코딩하지 않아 서버 목록이 바뀌어도 모든 위치가 표시된다.
export function splitLocations(locations: Location[]): { gates: Location[]; others: Location[] } {
  const num = (l: Location) => Number(/^gate\s*(\d+)$/i.exec(l.name.trim())?.[1] ?? NaN);
  const gates = locations.filter((l) => !Number.isNaN(num(l))).sort((a, b) => num(a) - num(b));
  const others = locations.filter((l) => Number.isNaN(num(l)));
  return { gates, others };
}

// 진행 단계. 서버 계약상 현재 오는 상태는 REQUESTED뿐이라 그 뒤 단계는 관제 연동 전까지 "예정"으로만 그린다.
export const PROGRESS_STEPS = ['예약 접수', '카트 배정', '운송 중', '수령 위치 도착', '수령 완료'] as const;

// 현재 단계 번호. 모르는 상태는 -1(어느 단계도 완료로 표시하지 않음).
export function progressIndex(status: string): number {
  return status === 'REQUESTED' ? 0 : -1;
}

// 한국 시간 HH:mm
export const hhmm = (iso: string) =>
  new Date(iso).toLocaleTimeString('ko-KR', { timeZone: 'Asia/Seoul', hour: '2-digit', minute: '2-digit', hourCycle: 'h23' });

// 오늘(KST)이면 HH:mm, 다른 날이면 M/D HH:mm. 자정을 넘는 수령 시간·항공편을 헷갈리지 않게 한다.
export function when(iso: string, now: Date = new Date()): string {
  const day = (d: Date) => d.toLocaleDateString('ko-KR', { timeZone: 'Asia/Seoul', month: 'numeric', day: 'numeric' });
  const d = new Date(iso);
  if (day(d) === day(now)) return hhmm(iso);
  const [m, dd] = d.toLocaleDateString('en-US', { timeZone: 'Asia/Seoul', month: 'numeric', day: 'numeric' }).split('/');
  return `${m}/${dd} ${hhmm(iso)}`;
}

// 탑승 알림 문구. 항공편의 탑승 시작·마감 시각만으로 계산한다(게이트 변경 실시간 반영은 관제·운항 연동 이후).
export function boardingNotice(flight: Pick<Flight, 'boardingStart' | 'boardingEnd'>, now: Date = new Date()): { tone: 'info' | 'warn' | 'done'; text: string } {
  const t = now.getTime();
  const start = new Date(flight.boardingStart).getTime();
  const end = new Date(flight.boardingEnd).getTime();
  if (t >= end) return { tone: 'done', text: '탑승이 마감되었습니다.' };
  if (t >= start) return { tone: 'warn', text: `탑승 중입니다. ${hhmm(flight.boardingEnd)}에 마감됩니다.` };
  const min = Math.ceil((start - t) / 60000);
  const left = min >= 60 ? `${Math.floor(min / 60)}시간${min % 60 ? ` ${min % 60}분` : ''}` : `${min}분`;
  return { tone: min <= 30 ? 'warn' : 'info', text: `탑승 시작까지 ${left} 남았습니다.` };
}
