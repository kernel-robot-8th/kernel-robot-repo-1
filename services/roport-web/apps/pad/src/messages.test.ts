// 실행(CI와 동일): services/roport-web 에서 `node --test 'apps/*/src/**/*.test.ts'`
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { FAILURE_TEXT, failureText, waitText } from './messages.ts';

test('대기 시간은 60초 미만은 초, 60초부터 분(올림)으로 말한다', () => {
  assert.equal(waitText(1), '1초');
  assert.equal(waitText(59), '59초');
  assert.equal(waitText(60), '1분');
  assert.equal(waitText(61), '2분');
  assert.equal(waitText(3600), '60분');
});

test('시도 제한 안내는 같은 앞 문장에 대기 시간만 바꿔 붙인다', () => {
  assert.equal(failureText('TOO_MANY_ATTEMPTS', 31), '확인에 여러 번 실패해 잠시 멈췄습니다. 약 31초 후 다시 시도해 주세요.');
  assert.equal(failureText('TOO_MANY_ATTEMPTS', 60), '확인에 여러 번 실패해 잠시 멈췄습니다. 약 1분 후 다시 시도해 주세요.');
  assert.equal(failureText('TOO_MANY_ATTEMPTS'), '확인에 여러 번 실패해 잠시 멈췄습니다. 잠시 후 다시 시도해 주세요.');
  // 다른 사유는 남은 시간이 와도 원래 문구를 쓴다.
  assert.equal(failureText('NOT_FOUND', 31), FAILURE_TEXT.NOT_FOUND);
});
