package com.roport.service.api;

import static org.assertj.core.api.Assertions.assertThat;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import org.junit.jupiter.api.Test;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.web.server.LocalServerPort;

/** 개발용 카트 인증이 꺼져 있으면(기본값) X-Cart-Id가 있어도 모든 /cart-api/** 가 401이다. */
@SpringBootTest(webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT)
class DevCartDisabledTest {
    @LocalServerPort
    int port;

    @Test
    void cartApiClosedByDefault() throws Exception {
        HttpResponse<String> r = HttpClient.newHttpClient().send(
                HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + "/cart-api/verifications"))
                        .header("Content-Type", "application/json").header("X-Cart-Id", "pinky3")
                        .POST(HttpRequest.BodyPublishers.ofString(
                                "{\"verificationRequestId\":\"rq-1\",\"method\":\"RESERVATION_NO\",\"value\":\"ABCDEFGH\"}"))
                        .build(),
                HttpResponse.BodyHandlers.ofString());
        assertThat(r.statusCode()).as(r.body()).isEqualTo(401);
        assertThat(r.body()).contains("\"code\":\"CART_UNAUTHENTICATED\"");
    }
}
