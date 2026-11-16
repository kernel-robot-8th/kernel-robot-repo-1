package com.roport.service.api;

import java.time.Clock;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.HashMap;
import java.util.Map;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpStatus;
import org.springframework.stereotype.Component;

/**
 * 카트별 예약 확인 실패(예약 없음·거절) 횟수 제한. 예약번호를 바꿔 가며 고객 정보를 조회하는 시도를 늦춘다.
 *
 * <p>기록은 서버 메모리에만 둔다(서버 1대 기준, 재시작하면 초기화). 카트 ID는 개발 전용 인증이라 위조할 수 있어
 * 카트별 제한만으로는 완전히 막지 못한다. 실제 카트 인증이 생기면 그 주체 기준으로 센다.
 */
@Component
class VerifyAttemptLimiter {
    /** 위조한 카트 ID를 바꿔 가며 보내도 메모리가 계속 늘지 않게, 이 수를 넘으면 지난 기록을 정리한다. */
    private static final int PRUNE_ABOVE = 1000;

    private final Clock clock;
    private final int maxFailures;
    private final Duration window;
    private final Map<String, Deque<Instant>> failures = new HashMap<>();

    VerifyAttemptLimiter(Clock clock,
            @Value("${service.verify.max-failures:5}") int maxFailures,
            @Value("${service.verify.failure-window:PT1M}") Duration window) {
        if (maxFailures < 1 || window.isNegative() || window.isZero()) {
            throw new IllegalArgumentException("service.verify.max-failures는 1 이상, failure-window는 양수여야 합니다.");
        }
        this.clock = clock;
        this.maxFailures = maxFailures;
        this.window = window;
    }

    /** 최근 실패가 기준에 이르렀으면 429. */
    synchronized void check(String cartId) {
        Deque<Instant> recent = failures.get(cartId);
        if (recent != null && trim(recent, clock.instant()) >= maxFailures) {
            throw new ApiException(HttpStatus.TOO_MANY_REQUESTS, "TOO_MANY_ATTEMPTS",
                    "예약 확인 실패가 많습니다. 잠시 후 다시 시도해 주세요.");
        }
    }

    synchronized void recordFailure(String cartId) {
        Instant now = clock.instant();
        failures.computeIfAbsent(cartId, k -> new ArrayDeque<>()).addLast(now);
        if (failures.size() > PRUNE_ABOVE) {
            failures.values().removeIf(d -> trim(d, now) == 0);
        }
    }

    private int trim(Deque<Instant> d, Instant now) {
        Instant from = now.minus(window);
        while (!d.isEmpty() && !d.peekFirst().isAfter(from)) {
            d.removeFirst();
        }
        return d.size();
    }
}
