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
        assertThatThrownBy(() -> limiter.check("c1"))
                .isInstanceOfSatisfying(ApiException.class, e -> assertThat(e.code).isEqualTo("TOO_MANY_ATTEMPTS"));
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
}
