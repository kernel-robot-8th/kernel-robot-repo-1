// 예약 확인 실패 안내 문구. 화면과 분리해 node --test로 검사한다.
import type { VerifyFailure } from './types';

// 시도 제한 안내의 앞 문장. 대기 시간이 있을 때와 없을 때 같은 문장을 쓴다.
const LIMIT_TEXT = '확인에 여러 번 실패해 잠시 멈췄습니다.';

export const FAILURE_TEXT: Record<VerifyFailure | 'ERROR', string> = {
  NOT_FOUND: '예약을 찾을 수 없습니다. 예약번호를 다시 확인해 주세요.',
  WRONG_CART: '이 카트에 배정된 예약이 아닙니다. 안내 직원에게 문의해 주세요.',
  ALREADY_RECEIVED: '이미 수령이 완료된 예약입니다.',
  NOT_ACTIVE: '지금 수령할 수 없는 예약입니다. 안내 직원에게 문의해 주세요.',
  TOO_MANY_ATTEMPTS: `${LIMIT_TEXT} 잠시 후 다시 시도해 주세요.`,
  ERROR: '서버와 통신하지 못했습니다. 잠시 후 다시 시도해 주세요.',
};

// 60초 미만은 초, 60초부터는 분(올림)으로 말한다.
export function waitText(seconds: number): string {
  return seconds < 60 ? `${seconds}초` : `${Math.ceil(seconds / 60)}분`;
}

// 시도 제한은 서버가 알려 준 남은 시간을 함께 안내한다. 남은 시간은 안내 시점 값이며 화면에서 세지 않는다
// (고객이 다시 입력하면 서버가 새 값을 준다).
export function failureText(reason: VerifyFailure | 'ERROR', retryAfterS?: number): string {
  if (reason !== 'TOO_MANY_ATTEMPTS' || retryAfterS === undefined) return FAILURE_TEXT[reason];
  return `${LIMIT_TEXT} 약 ${waitText(retryAfterS)} 후 다시 시도해 주세요.`;
}
