package com.roport.service.api;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.time.Clock;
import java.time.Duration;
import java.time.Instant;
import java.time.ZoneId;
import java.time.ZoneOffset;
import org.junit.jupiter.api.Test;

/** 실패 창(window)이 지나면 다시 허용하는지. 시계를 직접 움직인다. */
class VerifyAttemptLimiterTest {
    static class MovingClock extends Clock {
        Instant now = Instant.parse("2030-01-01T00:00:00Z");

        @Override public ZoneId getZone() { return ZoneOffset.UTC; }
        @Override public Clock withZone(ZoneId zone) { return this; }
        @Override public Instant instant() { return now; }
    }

    @Test
    void blocksAtLimitAndReopensAfterWindow() {
        MovingClock clock = new MovingClock();
        VerifyAttemptLimiter limiter = new VerifyAttemptLimiter(clock, 2, Duration.ofMinutes(1));
        limiter.check("c1");
        limiter.recordFailure("c1");
        clock.now = clock.now.plusSeconds(30);
        limiter.recordFailure("c1");
        // 첫 실패(0초)가 1분 창을 벗어나는 60초까지 30초 남음 → Retry-After 31(올림)
        assertThatThrownBy(() -> limiter.check("c1"))
                .isInstanceOfSatisfying(VerifyAttemptLimiter.TooManyAttempts.class, e -> {
                    assertThat(e.code).isEqualTo("TOO_MANY_ATTEMPTS");
                    assertThat(e.retryAfterSeconds).isEqualTo(31);
                });
        limiter.check("c2"); // 다른 카트는 별개

        clock.now = clock.now.plusSeconds(31); // 첫 실패가 1분 창을 벗어남
        limiter.check("c1");
    }

    @Test
    void rejectsInvalidSettings() {
        assertThatThrownBy(() -> new VerifyAttemptLimiter(Clock.systemUTC(), 0, Duration.ofMinutes(1)))
                .isInstanceOf(IllegalArgumentException.class);
        assertThatThrownBy(() -> new VerifyAttemptLimiter(Clock.systemUTC(), 5, Duration.ZERO))
                .isInstanceOf(IllegalArgumentException.class);
    }

    @Test
    void prunesAtMostOncePerWindow() {
        MovingClock clock = new MovingClock();
        VerifyAttemptLimiter limiter = new VerifyAttemptLimiter(clock, 5, Duration.ofMinutes(1));
        for (int i = 0; i < 1001; i++) {
            limiter.recordFailure("old-" + i);
        }
        clock.now = clock.now.plusSeconds(61); // 위 기록은 모두 창 밖
        limiter.recordFailure("new-0"); // 1002개 → 정리(창 밖 1001개 삭제)
        assertThat(limiter.trackedCarts()).isEqualTo(1);
        for (int i = 1; i <= 1001; i++) {
            limiter.recordFailure("new-" + i); // 같은 창 안에서는 다시 정리하지 않는다
        }
        assertThat(limiter.trackedCarts()).isEqualTo(1002);
        clock.now = clock.now.plusSeconds(61);
        limiter.recordFailure("later");
        assertThat(limiter.trackedCarts()).isEqualTo(1);
    }
}
