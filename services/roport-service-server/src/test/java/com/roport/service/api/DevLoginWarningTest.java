package com.roport.service.api;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.mock;

import ch.qos.logback.classic.Level;
import ch.qos.logback.classic.Logger;
import ch.qos.logback.classic.spi.ILoggingEvent;
import ch.qos.logback.core.read.ListAppender;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.slf4j.LoggerFactory;
import org.springframework.boot.test.context.runner.ApplicationContextRunner;
import org.springframework.jdbc.core.simple.JdbcClient;

/** 개발용 로그인이 켜졌을 때만 시작 시 WARN 을 한 번 남긴다. 전체 부팅 대신 SessionController 만 올려 로그를 안정적으로 캡처한다. */
class DevLoginWarningTest {
    final Logger logger = (Logger) LoggerFactory.getLogger(SessionController.class);
    final ListAppender<ILoggingEvent> logs = new ListAppender<>();
    final ApplicationContextRunner runner = new ApplicationContextRunner()
            .withBean(JdbcClient.class, () -> mock(JdbcClient.class))
            .withUserConfiguration(SessionController.class);

    @BeforeEach
    void attach() {
        logs.start();
        logger.addAppender(logs);
    }

    @AfterEach
    void detach() {
        logger.detachAppender(logs);
    }

    List<ILoggingEvent> warnings() {
        return logs.list.stream().filter(e -> e.getLevel() == Level.WARN).toList();
    }

    @Test
    void warnsOnceWhenEnabled() {
        runner.withPropertyValues("service.dev-login.enabled=true").run(ctx -> {
            assertThat(ctx).hasSingleBean(SessionController.class);
            assertThat(warnings()).singleElement()
                    .extracting(ILoggingEvent::getFormattedMessage).asString()
                    .contains("개발 전용 로그인").contains("공개 배포 금지");
        });
    }

    @Test
    void silentWhenDisabledOrUnset() {
        runner.withPropertyValues("service.dev-login.enabled=false")
                .run(ctx -> assertThat(ctx).doesNotHaveBean(SessionController.class));
        runner.run(ctx -> assertThat(ctx).doesNotHaveBean(SessionController.class));
        assertThat(warnings()).isEmpty();
    }
}
