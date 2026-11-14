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
  | { name: 'verifyFailed'; reason: VerifyFailure | 'ERROR' }
  | { name: 'items'; pickup: VerifiedPickup }
  | { name: 'chooseService'; pickup: VerifiedPickup }
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
        return goTo(s, r.ok ? { name: 'items', pickup: r.pickup } : { name: 'verifyFailed', reason: r.reason });
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
