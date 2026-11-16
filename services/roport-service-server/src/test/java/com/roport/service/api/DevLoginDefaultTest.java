package com.roport.service.api;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.BeforeAll;
import org.springframework.boot.test.context.SpringBootTest;

/**
 * properties override 없이 application.properties 기본값만으로 개발용 로그인이 꺼져 있는지 본다
 * (DevLoginDisabledTest 는 값을 직접 넣어 기본값이 true 로 바뀌어도 통과한다). 검사는 상속한 noWayToLogin.
 */
@SpringBootTest(webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT)
class DevLoginDefaultTest extends DevLoginDisabledTest {
    @BeforeAll
    static void requireNoOverride() {
        // 환경변수·시스템 속성이 기본값을 덮으면 이 테스트는 기본값을 볼 수 없다. 건너뛰지 않고 이유를 밝혀 실패한다.
        assertThat(System.getenv("SERVICE_DEV_LOGIN"))
                .as("SERVICE_DEV_LOGIN 환경변수가 설정돼 있어 기본값을 검사할 수 없다. 해제하고 실행하라").isNull();
        assertThat(System.getenv("SERVICE_DEVLOGIN_ENABLED"))
                .as("SERVICE_DEVLOGIN_ENABLED 환경변수가 service.dev-login.enabled 를 덮는다. 해제하고 실행하라").isNull();
        assertThat(System.getProperty("service.dev-login.enabled"))
                .as("service.dev-login.enabled 시스템 속성이 기본값을 덮는다. 해제하고 실행하라").isNull();
    }
}
