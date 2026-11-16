package com.roport.service.api;

import static org.assertj.core.api.Assertions.assertThat;

import java.net.CookieManager;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.UUID;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.web.server.LocalServerPort;
import org.springframework.context.ApplicationContext;
import org.springframework.jdbc.core.simple.JdbcClient;

/** 개발용 로그인이 꺼져 있으면(기본값) 세션을 만들 방법이 없고 모든 /api/** 가 401이다. */
@SpringBootTest(webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT, properties = "service.dev-login.enabled=false")
class DevLoginDisabledTest {
    @LocalServerPort
    int port;
    @Autowired
    JdbcClient db;
    @Autowired
    ApplicationContext ctx;

    HttpResponse<String> call(HttpClient c, String method, String path, String body) throws Exception {
        return c.send(HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + path))
                .header("Content-Type", "application/json")
                .method(method, body == null ? HttpRequest.BodyPublishers.noBody() : HttpRequest.BodyPublishers.ofString(body))
                .build(), HttpResponse.BodyHandlers.ofString());
    }

    static void unauthenticated(HttpResponse<String> r) {
        assertThat(r.statusCode()).as(r.body()).isEqualTo(401);
        assertThat(r.body()).contains("\"code\":\"UNAUTHENTICATED\"");
        assertThat(r.headers().firstValue("Set-Cookie")).as("세션 쿠키가 발급되면 안 된다").isEmpty();
    }

    @Test
    void noWayToLogin() throws Exception {
        String s = UUID.randomUUID().toString().substring(0, 8);
        db.sql("INSERT INTO members (member_id, login_id, display_name) VALUES (?, ?, 'D')")
                .param("md" + s).param("dis" + s).update();
        HttpClient c = HttpClient.newBuilder().cookieHandler(new CookieManager()).build();

        assertThat(ctx.getBeanNamesForType(SessionController.class)).isEmpty();
        // 인터셉터 면제 경로였던 POST /api/session 도 실제 회원 login_id 로 인증되지 않는다.
        unauthenticated(call(c, "POST", "/api/session", "{\"loginId\":\"dis" + s + "\"}"));
        unauthenticated(call(c, "GET", "/api/session", null));
        unauthenticated(call(c, "DELETE", "/api/session", null));
        unauthenticated(call(c, "GET", "/api/orders", null));
        unauthenticated(call(c, "GET", "/api/pickups", null));
        unauthenticated(call(c, "POST", "/api/pickups", "{}"));
        unauthenticated(call(c, "GET", "/api/no-such-path", null));
    }
}
