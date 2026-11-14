package com.roport.service.api;

import static org.assertj.core.api.Assertions.assertThat;

import java.net.CookieManager;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.UUID;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.web.server.LocalServerPort;
import org.springframework.jdbc.core.simple.JdbcClient;
import tools.jackson.databind.JsonNode;
import tools.jackson.databind.ObjectMapper;

/** 예약 QR 발급(/api)과 카트 예약 확인(/cart-api). 실제 PostgreSQL 격리 테스트 DB에 실행별 고유 ID의 가짜 행을 쓴다. */
@SpringBootTest(webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT,
        properties = {"service.dev-login.enabled=true", "service.dev-cart.enabled=true"})
class QrVerificationApiTest {
    @LocalServerPort
    int port;
    @Autowired
    JdbcClient db;
    @Autowired
    ObjectMapper json;

    String s;
    HttpClient alice;
    HttpClient bob;
    HttpClient cart;
    String reservationNo;
    String receivedNo;

    record Res(int status, String cacheControl, JsonNode body) {}

    @BeforeEach
    void seed() throws Exception {
        s = UUID.randomUUID().toString().substring(0, 8);
        alice = HttpClient.newBuilder().cookieHandler(new CookieManager()).build();
        bob = HttpClient.newBuilder().cookieHandler(new CookieManager()).build();
        cart = HttpClient.newHttpClient();
        // 예약번호는 서버 문자 집합(0·O·1·I 제외) 8자리. 실행별로 겹치지 않게 접미사에서 만든다.
        reservationNo = code("A");
        receivedNo = code("B");
        db.sql("INSERT INTO members (member_id, login_id, display_name) VALUES (?, ?, '테스트 A'), (?, ?, '테스트 B')")
                .param("ma" + s).param("alice" + s).param("mb" + s).param("bob" + s).update();
        db.sql("""
                INSERT INTO orders (order_id, order_no, member_id, site_id, revision, business_status, ordered_at) VALUES
                (?, ?, ?, 'DEV-SITE', 0, 'CONFIRMED', '2029-12-31T00:00Z'),
                (?, ?, ?, 'DEV-SITE', 0, 'RECEIVED', '2029-12-30T00:00Z')""")
                .param("oa" + s).param("NA" + s).param("ma" + s)
                .param("or" + s).param("NR" + s).param("ma" + s).update();
        db.sql("INSERT INTO order_items (item_id, order_id, product_name, quantity) VALUES (?, ?, 'test-item', 2)")
                .param("ia" + s).param("oa" + s).update();
        db.sql("INSERT INTO pickup_locations (location_id, site_id, name, control_point_ref, is_active) VALUES (?, 'DEV-SITE', 'test-gate', ?, true)")
                .param("la" + s).param("DEV-a" + s).update();
        db.sql("""
                INSERT INTO flights (flight_id, flight_no, service_date, boarding_start, boarding_end, departure_at, gate_code, revision)
                VALUES (?, ?, '2030-01-01', '2030-01-01T03:00Z', '2030-01-01T03:30Z', '2030-01-01T03:50Z', 'G2', 0)""")
                .param("fa" + s).param("T" + s).update();
        db.sql("""
                INSERT INTO pickups (pickup_id, reservation_no, order_id, location_id, flight_id, flight_revision,
                    flight_snapshot, pickup_at, channel, status, revision) VALUES
                (?, ?, ?, ?, ?, 0, '{}', '2030-01-01T02:00Z', 'WEB', 'REQUESTED', 0),
                (?, ?, ?, ?, ?, 0, '{}', '2029-12-30T02:00Z', 'WEB', 'CLOSED', 0)""")
                .param("pa" + s).param(reservationNo).param("oa" + s).param("la" + s).param("fa" + s)
                .param("pr" + s).param(receivedNo).param("or" + s).param("la" + s).param("fa" + s).update();
        login(alice, "alice" + s);
        login(bob, "bob" + s);
    }

    String code(String head) {
        StringBuilder b = new StringBuilder(head);
        for (char c : s.toUpperCase().toCharArray()) {
            if (b.length() < 8 && PickupService.RESERVATION_ALPHABET.indexOf(c) >= 0) b.append(c);
        }
        while (b.length() < 8) b.append('Z');
        return b.toString();
    }

    Res call(HttpClient c, String method, String path, String body, String cartId) throws Exception {
        HttpRequest.Builder b = HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + path))
                .header("Content-Type", "application/json")
                .method(method, body == null ? HttpRequest.BodyPublishers.noBody() : HttpRequest.BodyPublishers.ofString(body));
        if (cartId != null) b.header("X-Cart-Id", cartId);
        HttpResponse<String> r = c.send(b.build(), HttpResponse.BodyHandlers.ofString());
        return new Res(r.statusCode(), r.headers().firstValue("Cache-Control").orElse(""),
                r.body().isEmpty() ? null : json.readTree(r.body()));
    }

    void login(HttpClient c, String loginId) throws Exception {
        assertThat(call(c, "POST", "/api/session", "{\"loginId\":\"" + loginId + "\"}", null).status()).isEqualTo(200);
    }

    Res issue(HttpClient c, String pickupId) throws Exception {
        return call(c, "POST", "/api/pickups/" + pickupId + "/qr-token", null, null);
    }

    Res verify(String requestId, String method, String value) throws Exception {
        return verifyAs("pinky3", requestId, method, value);
    }

    Res verifyAs(String cartId, String requestId, String method, String value) throws Exception {
        String body = json.writeValueAsString(java.util.Map.of("verificationRequestId", requestId, "method", method, "value", value));
        return call(cart, "POST", "/cart-api/verifications", body, cartId);
    }

    static void problem(Res r, int status, String code) {
        assertThat(r.status()).as(String.valueOf(r.body())).isEqualTo(status);
        assertThat(r.body().path("code").asString("")).as(String.valueOf(r.body())).isEqualTo(code);
    }

    @Test
    void issueStoresOnlyHashAndRevokesPrevious() throws Exception {
        Res first = issue(alice, "pa" + s);
        assertThat(first.status()).as(String.valueOf(first.body())).isEqualTo(201);
        assertThat(first.cacheControl()).contains("no-store");
        String token = first.body().path("token").asString("");
        assertThat(token).startsWith(QrTokens.PREFIX).hasSizeGreaterThan(40);
        assertThat(first.body().path("pickupId").asString("")).isEqualTo("pa" + s);

        // DB에는 원문이 없고 해시만 있다.
        assertThat(db.sql("SELECT count(*) FROM qr_credentials WHERE token_hash = ?").param(token).query(Integer.class).single()).isZero();
        assertThat(db.sql("SELECT pickup_id FROM qr_credentials WHERE token_hash = ?").param(QrTokens.hash(token))
                .query(String.class).single()).isEqualTo("pa" + s);

        String second = issue(alice, "pa" + s).body().path("token").asString("");
        assertThat(second).isNotEqualTo(token);
        assertThat(db.sql("SELECT count(*) FROM qr_credentials WHERE pickup_id = ? AND revoked_at IS NULL")
                .param("pa" + s).query(Integer.class).single()).isEqualTo(1);

        Res old = verify("rq-old-" + s, "QR", token);
        assertThat(old.status()).isEqualTo(201);
        assertThat(old.body().path("result").asString("")).isEqualTo("REJECTED");
        assertThat(old.body().path("reasonCode").asString("")).isEqualTo("QR_REVOKED");
        assertThat(old.body().path("pickup").isNull()).isTrue();

        Res ok = verify("rq-new-" + s, "QR", second);
        assertThat(ok.status()).isEqualTo(201);
        assertThat(ok.body().path("result").asString("")).isEqualTo("VERIFIED");
        assertThat(db.sql("SELECT credential_id IS NOT NULL FROM verifications WHERE verification_request_id = ?")
                .param("rq-new-" + s).query(Boolean.class).single()).isTrue();
    }

    @Test
    void issueOnlyForOwnActivePickup() throws Exception {
        problem(issue(bob, "pa" + s), 404, "PICKUP_NOT_FOUND"); // 다른 회원의 예약은 존재를 드러내지 않는다
        problem(issue(alice, "no-such-" + s), 404, "PICKUP_NOT_FOUND");
        problem(issue(alice, "pr" + s), 409, "PICKUP_NOT_ACTIVE"); // 수령 완료(CLOSED) 예약
        problem(issue(HttpClient.newHttpClient(), "pa" + s), 401, "UNAUTHENTICATED");
    }

    @Test
    void verifyByReservationNoIsRecordedButNotHandoverApproval() throws Exception {
        Res r = verify("rq-a-" + s, "RESERVATION_NO", reservationNo.toLowerCase());
        assertThat(r.status()).as(String.valueOf(r.body())).isEqualTo(201);
        JsonNode b = r.body();
        assertThat(b.path("result").asString("")).isEqualTo("VERIFIED");
        assertThat(b.path("reasonCode").isNull()).isTrue();
        assertThat(b.path("cartAssignmentChecked").asBoolean(true)).isFalse();
        JsonNode p = b.path("pickup");
        assertThat(p.path("reservationNo").asString("")).isEqualTo(reservationNo);
        assertThat(p.path("customerName").asString("")).isEqualTo("테스트 A");
        assertThat(p.path("flight").path("gateCode").asString("")).isEqualTo("G2");
        assertThat(p.path("items").get(0).path("quantity").asInt()).isEqualTo(2);
        assertThat(p.path("packageCount").asInt(-1)).isZero();

        assertThat(db.sql("SELECT actor_ref || ' ' || method || ' ' || result FROM verifications WHERE verification_request_id = ?")
                .param("rq-a-" + s).query(String.class).single()).isEqualTo("cart:pinky3 RESERVATION_NO VERIFIED");
        // 확인은 인도 승인·수령이 아니다.
        assertThat(db.sql("SELECT count(*) FROM handover_approvals WHERE pickup_id = ?").param("pa" + s).query(Integer.class).single()).isZero();
        assertThat(db.sql("SELECT business_status FROM orders WHERE order_id = ?").param("oa" + s).query(String.class).single()).isEqualTo("CONFIRMED");
    }

    @Test
    void sameRequestIdReturnsStoredResultOrConflicts() throws Exception {
        Res first = verify("rq-dup-" + s, "RESERVATION_NO", reservationNo);
        Res again = verify("rq-dup-" + s, "RESERVATION_NO", reservationNo);
        assertThat(again.status()).isEqualTo(200);
        assertThat(again.body().path("verificationId").asString("")).isEqualTo(first.body().path("verificationId").asString("x"));
        assertThat(db.sql("SELECT count(*) FROM verifications WHERE verification_request_id = ?").param("rq-dup-" + s)
                .query(Integer.class).single()).isEqualTo(1);

        problem(verify("rq-dup-" + s, "RESERVATION_NO", receivedNo), 409, "IDEMPOTENCY_CONFLICT"); // 다른 예약
        problem(verifyAs("pinky2", "rq-dup-" + s, "RESERVATION_NO", reservationNo), 409, "IDEMPOTENCY_CONFLICT"); // 다른 카트

        // 같은 예약이라도 다른 QR 토큰이면 다른 요청이다(재발급 전 토큰의 결과를 새 토큰에 돌려주지 않는다).
        String a = issue(alice, "pa" + s).body().path("token").asString("");
        assertThat(verify("rq-qr-" + s, "QR", a).status()).isEqualTo(201);
        String b = issue(alice, "pa" + s).body().path("token").asString("");
        problem(verify("rq-qr-" + s, "QR", b), 409, "IDEMPOTENCY_CONFLICT");
        assertThat(verify("rq-qr-" + s, "QR", a).status()).isEqualTo(200); // 원래 토큰 재전송은 저장 결과
    }

    @Test
    void rejectionsAndNotFound() throws Exception {
        Res received = verify("rq-r-" + s, "RESERVATION_NO", receivedNo);
        assertThat(received.status()).isEqualTo(201);
        assertThat(received.body().path("result").asString("")).isEqualTo("REJECTED");
        assertThat(received.body().path("reasonCode").asString("")).isEqualTo("ALREADY_RECEIVED");

        problem(verify("rq-n-" + s, "RESERVATION_NO", "ZZZZZZZZ"), 404, "RESERVATION_NOT_FOUND");
        problem(verify("rq-q-" + s, "QR", QrTokens.newToken()), 404, "RESERVATION_NOT_FOUND");
        assertThat(db.sql("SELECT count(*) FROM verifications WHERE verification_request_id IN (?, ?)")
                .param("rq-n-" + s).param("rq-q-" + s).query(Integer.class).single()).isZero();
    }

    @Test
    void inputValidationAndCartAuth() throws Exception {
        problem(verify("rq-v1-" + s, "FACE", reservationNo), 400, "VALIDATION_FAILED");
        problem(verify("rq-v2-" + s, "RESERVATION_NO", "SHORT"), 400, "VALIDATION_FAILED");
        problem(verify("rq-v3-" + s, "RESERVATION_NO", "0O1IABCD"), 400, "VALIDATION_FAILED");
        problem(verify("rq-v4-" + s, "QR", "not-a-token"), 400, "VALIDATION_FAILED");
        problem(verify("bad id with space", "RESERVATION_NO", reservationNo), 400, "VALIDATION_FAILED");

        problem(verifyAs(null, "rq-c1-" + s, "RESERVATION_NO", reservationNo), 401, "CART_UNAUTHENTICATED");
        problem(verifyAs("bad cart!", "rq-c2-" + s, "RESERVATION_NO", reservationNo), 401, "CART_UNAUTHENTICATED");
        // 고객 세션 쿠키로는 카트 API에 들어갈 수 없다.
        problem(call(alice, "POST", "/cart-api/verifications", "{}", null), 401, "CART_UNAUTHENTICATED");
        // 카트 헤더로는 고객 API에 들어갈 수 없다.
        problem(call(cart, "GET", "/api/orders", null, "pinky3"), 401, "UNAUTHENTICATED");
    }
}
