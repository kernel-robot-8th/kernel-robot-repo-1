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
 * ID를 바꿔 가며 제한을 피하거나, 반대로 실제 패드의 ID로 실패를 쌓아 그 패드를 잠글 수 있다.
 * 실제 카트 인증이 생기면 그 주체 기준으로 센다.
 */
@Component
class VerifyAttemptLimiter {
    /** 위조한 카트 ID를 바꿔 가며 보내도 메모리가 계속 늘지 않게, 이 수를 넘으면 지난 기록을 정리한다(창마다 최대 1번). */
    private static final int PRUNE_ABOVE = 1000;

    private final Clock clock;
    private final int maxFailures;
    private final Duration window;
    private final Map<String, Deque<Instant>> failures = new HashMap<>();
    private Instant nextPruneAt = Instant.MIN;

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

    /** 최근 실패가 기준에 이르렀으면 429. Retry-After는 기준 아래로 내려가는 데 남은 초다. */
    synchronized void check(String cartId) {
        Deque<Instant> recent = failures.get(cartId);
        Instant now = clock.instant();
        if (recent != null && trim(recent, now) >= maxFailures) {
            // 가장 오래된 실패부터 창을 벗어난다. 기준 아래로 내려가려면 (개수 - 기준 + 1)번째 실패가 빠져야 한다.
            Instant reopenAt = recent.stream().skip(recent.size() - maxFailures).findFirst().orElseThrow().plus(window);
            long seconds = Math.max(1, Duration.between(now, reopenAt).toMillis() / 1000 + 1);
            throw new TooManyAttempts(seconds);
        }
    }

    /** 카트 확인 실패 제한. ApiExceptionHandler가 Retry-After 헤더를 붙인다. */
    static final class TooManyAttempts extends ApiException {
        final long retryAfterSeconds;

        TooManyAttempts(long retryAfterSeconds) {
            super(HttpStatus.TOO_MANY_REQUESTS, "TOO_MANY_ATTEMPTS", "예약 확인 실패가 많습니다. 잠시 후 다시 시도해 주세요.");
            this.retryAfterSeconds = retryAfterSeconds;
        }
    }

    synchronized void recordFailure(String cartId) {
        Instant now = clock.instant();
        failures.computeIfAbsent(cartId, k -> new ArrayDeque<>()).addLast(now);
        if (failures.size() > PRUNE_ABOVE && !now.isBefore(nextPruneAt)) {
            // 창 안의 카트 ID가 많으면 지울 것이 없어도 매번 전체를 훑게 되므로 창마다 한 번만 정리한다.
            failures.values().removeIf(d -> trim(d, now) == 0);
            nextPruneAt = now.plus(window);
        }
    }

    /** 시험용: 기록 중인 카트 수. */
    synchronized int trackedCarts() {
        return failures.size();
    }

    private int trim(Deque<Instant> d, Instant now) {
        Instant from = now.minus(window);
        while (!d.isEmpty() && !d.peekFirst().isAfter(from)) {
            d.removeFirst();
        }
        return d.size();
    }
}
