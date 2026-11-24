// 패드 화면 상태 전이. 화면(App.tsx)과 분리해 node --test로 검사한다.
//
// 두 가지 원칙을 지킨다.
// 1. 요청 응답은 그 요청을 보낸 뒤 화면이 바뀌지 않았을 때만 반영한다.
//    요청마다 토큰을 붙이고, 화면 전환·초기화는 진행 중 토큰을 지운다. 토큰이 다른 응답은 버린다.
// 2. 출발·중지·재개 요청의 accepted는 "접수"일 뿐이다. 카트가 실제로 움직이는지·멈췄는지는
//    카트 상태 보고(robotState)로만 바꾼다. 지금은 데모 바의 모의 상태 입력이 그 보고를 대신한다.
import type { ServiceMode, VerifiedPickup, VerifyFailure, VerifyResult } from './types';

// 카트가 보고한 실제 상태. WAITING은 출발 접수 후 아직 상태 보고가 없는 경우.
export type Motion = 'WAITING' | 'MOVING' | 'PAUSED';
// 접수됐지만 카트 상태로 아직 확인되지 않은 명령
export type PendingCommand = 'START' | 'PAUSE' | 'RESUME' | null;

export type MovingStep = {
  name: 'moving';
  pickup: VerifiedPickup;
  mode: ServiceMode;
  motion: Motion;
  pending: PendingCommand;
  arrived: boolean;
};

export type Step =
  | { name: 'idle' }
  | { name: 'keypad' }
  | { name: 'verifying' }
  | { name: 'verifyFailed'; reason: VerifyFailure | 'ERROR'; retryAfterS?: number }
  | { name: 'items'; pickup: VerifiedPickup }
  | { name: 'chooseService'; pickup: VerifiedPickup }
  | { name: 'shopping'; pickup: VerifiedPickup }
  | MovingStep
  | { name: 'receipt'; pickup: VerifiedPickup }
  | { name: 'returning' };

export type Notice = 'ERROR' | 'SERVICE_REJECTED' | 'COMMAND_REJECTED' | 'RECEIPT_NOT_SAVED' | null;

export type PadState = {
  step: Step;
  request: number | null; // 응답을 기다리는 요청 토큰. null이면 대기 중인 요청 없음
  stopping: boolean; // 기다리는 요청이 중지 요청인지(중지 버튼 중복 방지)
  notice: Notice;
  lastPickup: VerifiedPickup | null; // 복귀 화면에서도 진행 표시줄을 그리기 위한 마지막 예약
};

type Accepted = { accepted: boolean } | null; // null: 통신 실패

export type Action =
  | { type: 'go'; step: Step } // 사용자 화면 전환
  | { type: 'reset' }
  // 요청 시작(필요하면 대기 화면으로). stop은 중지 요청: 다른 요청을 기다리는 중에도 보낼 수 있고, 그 요청의 응답은 버린다.
  | { type: 'send'; token: number; step?: Step; stop?: boolean }
  | { type: 'verified'; token: number; result: VerifyResult | null }
  | { type: 'serviceAnswered'; token: number; pickup: VerifiedPickup; mode: ServiceMode; res: Accepted }
  | { type: 'commandAnswered'; token: number; command: 'PAUSE' | 'RESUME'; res: Accepted }
  | { type: 'receiptAnswered'; token: number; res: Accepted }
  | { type: 'robotState'; motion: 'MOVING' | 'PAUSED' } // 카트 상태 보고(데모: 모의 입력)
  | { type: 'arrived' }; // 게이트 도착 보고(데모: 모의 입력)

export const initialState: PadState = { step: { name: 'idle' }, request: null, stopping: false, notice: null, lastPickup: null };

const pickupOf = (step: Step): VerifiedPickup | null => ('pickup' in step ? step.pickup : null);

// 화면 전환: 진행 중 요청을 무효로 하고 안내 문구를 지운다.
function goTo(state: PadState, step: Step): PadState {
  return { ...state, step, request: null, stopping: false, notice: null, lastPickup: pickupOf(step) ?? state.lastPickup };
}

// 토큰이 현재 요청과 같을 때만 응답을 반영한다.
function answer(state: PadState, token: number, apply: (s: PadState) => PadState): PadState {
  if (state.request !== token) return state;
  return apply({ ...state, request: null, stopping: false });
}

export function reduce(state: PadState, action: Action): PadState {
  switch (action.type) {
    case 'go':
      return goTo(state, action.step);
    case 'reset':
      return initialState;
    case 'send':
      // 한 번에 요청 하나. 중지만 예외로 앞 요청을 대신하되, 중지를 기다리는 중의 중복 중지는 받지 않는다.
      if (state.request !== null && (!action.stop || state.stopping)) return state;
      return { ...(action.step ? goTo(state, action.step) : state), request: action.token, stopping: Boolean(action.stop), notice: null };
    case 'verified':
      return answer(state, action.token, (s) => {
        const r = action.result;
        if (!r) return goTo(s, { name: 'verifyFailed', reason: 'ERROR' });
        if (r.ok) return goTo(s, { name: 'items', pickup: r.pickup });
        return goTo(s, { name: 'verifyFailed', reason: r.reason, ...(r.retryAfterS !== undefined && { retryAfterS: r.retryAfterS }) });
      });
    case 'serviceAnswered':
      return answer(state, action.token, (s) => {
        if (!action.res) return { ...s, notice: 'ERROR' };
        if (!action.res.accepted) return { ...s, notice: 'SERVICE_REJECTED' };
        return goTo(s, { name: 'moving', pickup: action.pickup, mode: action.mode, motion: 'WAITING', pending: 'START', arrived: false });
      });
    case 'commandAnswered':
      return answer(state, action.token, (s) => {
        if (s.step.name !== 'moving') return s;
        if (!action.res) return { ...s, notice: 'ERROR' };
        if (!action.res.accepted) return { ...s, notice: 'COMMAND_REJECTED' };
        return { ...s, step: { ...s.step, pending: action.command } };
      });
    case 'receiptAnswered':
      return answer(state, action.token, (s) => {
        if (!action.res) return { ...s, notice: 'ERROR' };
        if (!action.res.accepted) return { ...s, notice: 'RECEIPT_NOT_SAVED' };
        return goTo(s, { name: 'returning' });
      });
    case 'robotState': {
      if (state.step.name !== 'moving') return state;
      const { pending } = state.step;
      // 보고된 상태가 기다리던 명령의 결과일 때만 대기 표시를 지운다.
      const done = action.motion === 'MOVING' ? pending === 'START' || pending === 'RESUME' : pending === 'PAUSE';
      return { ...state, step: { ...state.step, motion: action.motion, pending: done ? null : pending } };
    }
    case 'arrived':
      if (state.step.name !== 'moving') return state;
      return { ...state, step: { ...state.step, arrived: true, pending: null } };
  }
}

// 이동 화면에 보여 줄 단계. 왼쪽 상태 표시·제목·버튼이 모두 이 값 하나에서 나온다.
export type MovingPhase = 'START_PENDING' | 'MOVING' | 'PAUSE_PENDING' | 'PAUSED' | 'RESUME_PENDING' | 'ARRIVED';

export function movingPhase(step: MovingStep): MovingPhase {
  if (step.arrived) return 'ARRIVED';
  if (step.pending) return `${step.pending}_PENDING`;
  return step.motion === 'PAUSED' ? 'PAUSED' : 'MOVING';
}

// 큰 버튼: 카트가 멈춘 것이 확인됐을 때만 재개, 그 밖에는 언제나 중지(출발·재개 확인 중에도 멈출 수 있게).
// 중지는 접수 뒤에도 카트가 멈춘 것이 확인될 때까지 다시 누를 수 있다.
export function bigButton(phase: MovingPhase): 'PAUSE' | 'RESUME' | null {
  if (phase === 'ARRIVED') return null;
  return phase === 'PAUSED' ? 'RESUME' : 'PAUSE';
}

/** A person at the centre can be selected when only clipped edge fragments compete. */
export function automaticShopper(boxes: number[][]): number | null {
  if (boxes.length === 1) return 0;
  const visible = boxes.map(([left,,right], index) => ({left, right, index, center: (left + right) / 2}))
    .filter(b => !((b.left <= .01 && b.center < .20) || (b.right >= .99 && b.center > .80)));
  if (visible.length === 1 && Math.abs(visible[0].center - .5) <= .15) return visible[0].index;
  return null;
}

/** 외형 추종은 명시적으로 켠 단독 시험 빌드의 데모 예약 화면에서만 열 수 있다. */
export function shoppingDemoEnabled(flag: string | undefined, apiMode: string | undefined): boolean {
  return flag === 'true' && apiMode !== 'server';
}

// Service follow state is authoritative; HTTP command acceptance never changes it.
export function followView(stage: string, status: string, mode: string): { title: string; lead: string } {
  if (status === 'HOLD') return { title: '카트 상태 확인이 필요합니다', lead: '현재 상태를 유지하고 있습니다. 안내 직원에게 문의해 주세요.' };
  if (status === 'CANCELED') return { title: '서비스가 중단되었습니다', lead: '카트 상태는 안내 직원에게 확인해 주세요.' };
  if (status === 'COMPLETE' || status === 'COMPLETED' || stage === 'done') return { title: '이용이 완료되었습니다', lead: '물품을 빠짐없이 챙겨 주세요.' };
  if (mode === 'none' && stage === 'follow_pending') return { title: '수령이 완료되었습니다', lead: '카트 복귀 준비 상태를 확인하고 있습니다.' };
  switch (stage) {
    case 'follow_register': return { title: '함께 갈 사람을 등록해 주세요', lead: '카트 앞에 서서 사진을 찍은 뒤, 사진에서 본인을 직접 선택해 주세요.' };
    case 'follow_registering': return { title: '대상을 등록하고 있습니다', lead: '선택한 모습을 확인하고 있습니다. 카트 앞에서 기다려 주세요.' };
    case 'follow_pending': return { title: '출발 승인을 기다리고 있습니다', lead: '관제에서 준비 상태를 확인한 뒤 출발합니다. 아직 따라가기 시작하지 않았습니다.' };
    case 'follow_start': return { title: '출발을 확인하고 있습니다', lead: '실제 추종 상태가 확인될 때까지 기다려 주세요.' };
    case 'following': return { title: '카트가 따라가고 있습니다', lead: mode === 'marker' ? '마커 카드가 카트 쪽을 향하도록 하고 천천히 걸어 주세요.' : '카트 앞에서 천천히 걸어 주세요. 멈추려면 아래 정지 버튼을 눌러 주세요.' };
    case 'follow_stopping': return { title: '정지를 확인하고 있습니다', lead: '카트가 완전히 멈출 때까지 가까이 가지 마세요.' };
    case 'follow_paused': return { title: '카트가 멈췄습니다', lead: '주변을 확인한 뒤 재개하거나, 물품을 모두 내리고 복귀를 요청해 주세요.' };
    case 'return_nav': case 'switch_line': case 'return_route': return { title: '카트가 복귀하고 있습니다', lead: '복귀 완료가 확인될 때까지 상태를 표시합니다.' };
    default: return { title: '카트 상태를 확인하고 있습니다', lead: '관제에서 준비 상태를 확인하고 있습니다.' };
  }
}
