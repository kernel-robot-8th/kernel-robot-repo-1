package com.roport.service.api;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.net.CookieManager;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.UUID;
import java.util.Map;
import java.util.List;
import java.time.Instant;
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
    @Autowired
    com.roport.service.control.ControlExchange exchange;

    String s;
    HttpClient alice;
    HttpClient bob;
    HttpClient cart;
    String reservationNo;
    String receivedNo;

    record Res(int status, String cacheControl, String retryAfter, JsonNode body) {}

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
                r.headers().firstValue("Retry-After").orElse(""),
                r.body().isEmpty() ? null : json.readTree(r.body()));
    }

    void login(HttpClient c, String loginId) throws Exception {
        assertThat(call(c, "POST", "/api/session", "{\"loginId\":\"" + loginId + "\"}", null).status()).isEqualTo(200);
    }

    Res issue(HttpClient c, String pickupId) throws Exception {
        return call(c, "POST", "/api/pickups/" + pickupId + "/qr-token", null, null);
    }

    /** 카트별 실패 제한이 테스트끼리 섞이지 않게 테스트마다 카트 ID를 따로 쓴다. */
    String cartId() {
        return "pinky3-" + s;
    }

    Res verify(String requestId, String method, String value) throws Exception {
        return verifyAs(cartId(), requestId, method, value);
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
        // 만료는 발급 시각 + 기본 10분(service.qr.ttl).
        java.time.Instant issuedAt = java.time.Instant.parse(first.body().path("issuedAt").asString(""));
        assertThat(java.time.Instant.parse(first.body().path("expiresAt").asString(""))).isEqualTo(issuedAt.plusSeconds(600));

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
                .param("rq-a-" + s).query(String.class).single()).isEqualTo("cart:" + cartId() + " RESERVATION_NO VERIFIED");
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
        // 이미 쓴 ID에 없는 값을 보내도 404가 아니라 409다(오류 우선순위). 처음 쓰는 ID의 없는 값은 404.
        problem(verify("rq-dup-" + s, "RESERVATION_NO", "ZZZZZZZZ"), 409, "IDEMPOTENCY_CONFLICT");
        problem(verify("rq-dup-" + s, "QR", QrTokens.newToken()), 409, "IDEMPOTENCY_CONFLICT");

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

    @Test
    void expiredQrIsRejected() throws Exception {
        String token = issue(alice, "pa" + s).body().path("token").asString("");
        assertThat(db.sql("SELECT expires_at > issued_at FROM qr_credentials WHERE token_hash = ?")
                .param(QrTokens.hash(token)).query(Boolean.class).single()).isTrue();
        db.sql("UPDATE qr_credentials SET expires_at = now() - interval '1 second' WHERE token_hash = ?")
                .param(QrTokens.hash(token)).update();
        Res expired = verify("rq-exp-" + s, "QR", token);
        assertThat(expired.status()).isEqualTo(201);
        assertThat(expired.body().path("result").asString("")).isEqualTo("REJECTED");
        assertThat(expired.body().path("reasonCode").asString("")).isEqualTo("QR_EXPIRED");
    }

    @Test
    void repeatedFailuresAreLimitedPerCart() throws Exception {
        assertThat(verify("rq-lim-ok-" + s, "RESERVATION_NO", reservationNo).status()).isEqualTo(201);
        // 기본 기준: 1분에 실패 5회(예약 없음 4회 + 거절 1회).
        for (int i = 1; i <= 4; i++) {
            problem(verify("rq-lim-" + i + "-" + s, "RESERVATION_NO", "ZZZZZZZZ"), 404, "RESERVATION_NOT_FOUND");
        }
        assertThat(verify("rq-lim-5-" + s, "RESERVATION_NO", receivedNo).body().path("reasonCode").asString(""))
                .isEqualTo("ALREADY_RECEIVED");
        // 기록된 거절의 재전송(200)은 다시 세지 않는다. 세었다면 아래 재전송 확인 전에 이미 막혔을 것이다.
        assertThat(verify("rq-lim-5-" + s, "RESERVATION_NO", receivedNo).status()).isEqualTo(200);

        // 기준에 이르면 올바른 예약번호라도 새 요청은 429.
        Res limited = verify("rq-lim-6-" + s, "RESERVATION_NO", reservationNo);
        problem(limited, 429, "TOO_MANY_ATTEMPTS");
        assertThat(Integer.parseInt(limited.retryAfter())).isBetween(1, 61);
        // 이미 기록된 요청의 재전송은 막지 않는다(패드의 응답 유실 재시도).
        assertThat(verify("rq-lim-ok-" + s, "RESERVATION_NO", reservationNo).status()).isEqualTo(200);
        // 다른 카트는 영향이 없다.
        assertThat(verifyAs("other-" + s, "rq-lim-other-" + s, "RESERVATION_NO", reservationNo).status()).isEqualTo(201);
    }

    @Test
    void replayOfExpiredQrDoesNotReadLatestPickup() throws Exception {
        String token = issue(alice, "pa" + s).body().path("token").asString("");
        Res first = verify("rq-rp-" + s, "QR", token);
        assertThat(first.status()).isEqualTo(201);
        assertThat(first.body().path("pickup").path("flight").path("gateCode").asString("")).isEqualTo("G2");

        // 만료 전 재시도: 저장된 결과와 함께 예약 정보를 준다(응답 유실 재시도).
        Res beforeExpiry = verify("rq-rp-" + s, "QR", token);
        assertThat(beforeExpiry.status()).isEqualTo(200);
        assertThat(beforeExpiry.body().path("pickup").path("flight").path("gateCode").asString("")).isEqualTo("G2");

        // 만료 후 게이트가 바뀌어도 재전송은 저장된 결과만 돌려주고 최신 예약 정보를 조회하지 않는다.
        db.sql("UPDATE qr_credentials SET expires_at = now() - interval '1 second' WHERE token_hash = ?")
                .param(QrTokens.hash(token)).update();
        db.sql("UPDATE flights SET gate_code = 'G9' WHERE flight_id = ?").param("fa" + s).update();
        Res afterExpiry = verify("rq-rp-" + s, "QR", token);
        assertThat(afterExpiry.status()).isEqualTo(200);
        assertThat(afterExpiry.body().path("result").asString("")).isEqualTo("VERIFIED");
        assertThat(afterExpiry.body().path("verificationId").asString("")).isEqualTo(first.body().path("verificationId").asString(""));
        assertThat(afterExpiry.body().path("pickup").isNull()).as(String.valueOf(afterExpiry.body())).isTrue();
    }

    void packageRow(String id) {
        db.sql("INSERT INTO packages(package_id,order_id,package_revision,label_ref,manifest_locked_at) VALUES(?,?,0,'test',now())")
                .param(id).param("oa" + s).update();
    }

    void arrived(String id) {
        String progress = json.writeValueAsString(java.util.Map.of("progress", java.util.Map.of("packages", java.util.List.of(
                java.util.Map.of("package_id", id, "robot", cartId(), "stage", "await_receipt")))));
        db.sql("""
                INSERT INTO control_task_views(task_ref,pickup_id,site_id,task_revision,state,progress_payload,observed_at,synced_at)
                VALUES(?,?,'DEV-SITE',1,'RUNNING',?::jsonb,now(),now()) ON CONFLICT(task_ref) DO UPDATE
                SET progress_payload=excluded.progress_payload,task_revision=control_task_views.task_revision+1
                """).param("task-" + s).param("pa" + s).param(progress).update();
    }

    Res receive(String verification, String request, String cartId) throws Exception {
        return call(cart, "POST", "/cart-api/receipts", json.writeValueAsString(java.util.Map.of(
                "verificationId", verification, "confirmationRequestId", request)), cartId);
    }

    @Test
    void receiptPreservesRequestedFollowCardOnRetry() throws Exception {
        packageRow("a-" + s);
        arrived("a-" + s);
        String verification = verify("verify-" + s, "RESERVATION_NO", reservationNo).body().path("verificationId").asString();
        var body = new java.util.HashMap<String, Object>(java.util.Map.of(
                "verificationId", verification, "confirmationRequestId", "follow-" + s, "followCard", 50));
        assertThat(call(cart, "POST", "/cart-api/receipts", json.writeValueAsString(body), cartId()).status()).isEqualTo(400);
        body.put("followCard", 42);
        var receipt = call(cart, "POST", "/cart-api/receipts", json.writeValueAsString(body), cartId());
        assertThat(receipt.status()).as(receipt.body().toString()).isEqualTo(200);
        assertThat(call(cart, "POST", "/cart-api/receipts", json.writeValueAsString(body), cartId()).body())
                .isEqualTo(receipt.body());
        assertThat(receive(verification, "follow-" + s, cartId()).status()).isEqualTo(409);
    }

    @Test
    void receiptRequiresExplicitConfirmationAndMatchingCartArrival() throws Exception {
        packageRow("a-" + s);
        String verified = verify("verify-" + s, "RESERVATION_NO", reservationNo).body().path("verificationId").asString();
        problem(receive(verified, "receive-" + s, cartId()), 409, "CART_NOT_ARRIVED");
        arrived("a-" + s);
        problem(receive(verified, "receive-" + s, "other"), 409, "VERIFICATION_REQUIRED");
        Res first = receive(verified, "receive-" + s, cartId());
        assertThat(first.status()).as(first.body().toString()).isEqualTo(200);
        assertThat(first.body().path("orderComplete").asBoolean()).isTrue();
        assertThat(receive(verified, "receive-" + s, cartId()).body().path("receiptId")).isEqualTo(first.body().path("receiptId"));
        assertThat(db.sql("SELECT count(*) FROM receipts WHERE order_id=?").param("oa" + s).query(Integer.class).single()).isEqualTo(1);
    }

    @Test
    void sequentialPackageReceiptsCompleteOrderOnlyAfterLastDelivery() throws Exception {
        packageRow("a-" + s); packageRow("b-" + s);
        String verified = verify("verify-" + s, "RESERVATION_NO", reservationNo).body().path("verificationId").asString();
        arrived("a-" + s);
        Res first = receive(verified, "receive-a-" + s, cartId());
        assertThat(first.status()).as(first.body().toString()).isEqualTo(200);
        assertThat(first.body().path("orderComplete").asBoolean()).isFalse();
        problem(receive(verified, "another-a-" + s, cartId()), 409, "PACKAGE_NOT_AVAILABLE");
        arrived("b-" + s);
        Res last = receive(verified, "receive-b-" + s, cartId());
        assertThat(last.status()).as(last.body().toString()).isEqualTo(200);
        assertThat(last.body().path("orderComplete").asBoolean()).isTrue();
        assertThat(db.sql("SELECT count(*) FROM receipts WHERE order_id=?").param("oa" + s).query(Integer.class).single()).isEqualTo(2);
    }

    @Test
    void expiredVerificationCannotConfirmHandover() throws Exception {
        packageRow("a-" + s); arrived("a-" + s);
        String verified = verify("verify-" + s, "RESERVATION_NO", reservationNo).body().path("verificationId").asString();
        db.sql("UPDATE verifications SET verified_at=now()-interval '10 minutes' WHERE verification_id=?").param(verified).update();
        problem(receive(verified, "receive-" + s, cartId()), 409, "VERIFICATION_REQUIRED");
    }

    @Test
    void manifestRefreshUsesStoredPackagesAndOwnership() throws Exception {
        packageRow("a-" + s);
        problem(call(bob, "POST", "/api/pickups/pa" + s + "/manifest-refresh", null, null), 404, "PICKUP_NOT_FOUND");
        Res result = call(alice, "POST", "/api/pickups/pa" + s + "/manifest-refresh", null, null);
        assertThat(result.status()).as(result.body().toString()).isEqualTo(200);
        assertThat(result.body().path("submitted").asBoolean()).isTrue();
        assertThat(call(alice, "POST", "/api/pickups/pa" + s + "/manifest-refresh", null, null).body().path("submitted").asBoolean()).isFalse();
    }

    void followProjection(String receipt, String stage, long revision, Instant observed, String state) {
        var item = new java.util.LinkedHashMap<String, Object>();
        item.put("package_id", "a-" + s); item.put("robot", cartId()); item.put("stage", stage);
        item.put("robot_observed_at", observed.toString());
        item.put("supported_follow_modes", List.of("marker", "appearance"));
        if (receipt != null) item.put("follow", Map.of("session_id", receipt, "mode", "appearance", "revision", revision,
                "reason", "", "observed_at", observed.toString(), "allowed_actions", List.of("capture", "select", "return"),
                "registration", Map.of("frame_ref", "frame-" + s, "image_ref", "private-robot-url-not-for-client",
                        "expires_at", Instant.now().plusSeconds(30).toString(), "perception_boot_id", "boot-test",
                        "candidates", List.of(Map.of("candidate_id", "person-1", "box", List.of(0.1, 0.2, 0.4, 0.8))))));
        db.sql("UPDATE control_task_views SET progress_payload=?::jsonb,state=?,synced_at=now() WHERE task_ref=?")
                .param(json.writeValueAsString(Map.of("progress", Map.of("packages", List.of(item)))))
                .param(state).param("task-" + s).update();
    }

    String appearanceReceipt() throws Exception {
        packageRow("a-" + s); arrived("a-" + s);
        followProjection(null, "await_receipt", 0, Instant.now(), "RUNNING");
        String verification = verify("verify-" + s, "RESERVATION_NO", reservationNo).body().path("verificationId").asString();
        var result = call(cart, "POST", "/cart-api/receipts", json.writeValueAsString(Map.of("verificationId", verification,
                "confirmationRequestId", "receipt-" + s, "followMode", "appearance")), cartId());
        assertThat(result.status()).as(result.body().toString()).isEqualTo(200);
        assertThat(result.body().path("followMode").asString()).isEqualTo("appearance");
        String receipt = result.body().path("receiptId").asString();
        followProjection(receipt, "follow_register", 3, Instant.now(), "RUNNING");
        return receipt;
    }

    Res followAction(String receipt, String request, long revision, String operation) throws Exception {
        return call(cart, "POST", "/cart-api/follow/" + receipt + "/actions", json.writeValueAsString(
                Map.of("requestId", request, "expectedRevision", revision, "operation", operation)), cartId());
    }

    @Test
    void explicitFollowModeRequiresFreshRobotCapabilityAndCompatibleCard() throws Exception {
        packageRow("a-" + s); arrived("a-" + s);
        String verification = verify("verify-" + s, "RESERVATION_NO", reservationNo).body().path("verificationId").asString();
        String path = "/cart-api/follow/capabilities?verificationId=" + verification;
        assertThat(call(cart, "GET", path, null, cartId()).body().path("modes").size()).isZero();
        var body = new java.util.HashMap<String, Object>(Map.of("verificationId", verification,
                "confirmationRequestId", "receipt-" + s, "followMode", "appearance"));
        problem(call(cart, "POST", "/cart-api/receipts", json.writeValueAsString(body), cartId()), 409, "FOLLOW_MODE_UNSUPPORTED");
        followProjection(null, "await_receipt", 0, Instant.now(), "RUNNING");
        assertThat(call(cart, "GET", path, null, cartId()).body().path("modes").toString()).isEqualTo("[\"marker\",\"appearance\"]");
        problem(call(cart, "GET", path, null, "other"), 409, "VERIFICATION_REQUIRED");
        body.put("followCard", 42);
        problem(call(cart, "POST", "/cart-api/receipts", json.writeValueAsString(body), cartId()), 400, "VALIDATION_FAILED");
        body.remove("followCard");
        followProjection(null, "await_receipt", 0, Instant.now().minusSeconds(4), "RUNNING");
        problem(call(cart, "POST", "/cart-api/receipts", json.writeValueAsString(body), cartId()), 409, "FOLLOW_MODE_UNSUPPORTED");
        // No-follow remains available without a follow executor, preserving receipt and return behavior.
        body.put("followMode", "none");
        var receipt = call(cart, "POST", "/cart-api/receipts", json.writeValueAsString(body), cartId());
        assertThat(receipt.status()).as(receipt.body().toString()).isEqualTo(200);
        assertThat(receipt.body().path("followMode").asString()).isEqualTo("none");
    }

    @Test
    void followStatusRestoresOnlyOwnedSessionAndFailsClosedOnStaleOrHold() throws Exception {
        String receipt = appearanceReceipt();
        String path = "/cart-api/follow/" + receipt;
        var status = call(cart, "GET", path, null, cartId());
        assertThat(status.status()).isEqualTo(200);
        assertThat(status.cacheControl()).contains("no-store");
        assertThat(status.body().path("stage").asString()).isEqualTo("follow_register");
        assertThat(status.body().path("revision").asLong()).isEqualTo(3);
        assertThat(status.body().path("stale").asBoolean()).isFalse();
        assertThat(status.body().path("registration").path("imagePath").asString()).isEqualTo(path + "/frames/frame-" + s);
        assertThat(status.body().toString()).doesNotContain("private-robot-url", "target_ref");
        assertThat(call(cart, "GET", path, null, cartId()).body()).isEqualTo(status.body());
        problem(call(cart, "GET", path, null, "other"), 404, "FOLLOW_NOT_FOUND");
        problem(call(cart, "GET", path, null, null), 401, "CART_UNAUTHENTICATED");
        followProjection(receipt, "follow_register", 3, Instant.now().minusSeconds(4), "RUNNING");
        assertThat(call(cart, "GET", path, null, cartId()).body().path("allowedActions").size()).isZero();
        problem(followAction(receipt, "capture-stale-" + s, 3, "capture"), 409, "FOLLOW_ACTION_UNAVAILABLE");
        followProjection(receipt, "follow_register", 3, Instant.now(), "HOLD");
        assertThat(call(cart, "GET", path, null, cartId()).body().path("status").asString()).isEqualTo("HOLD");
        problem(followAction(receipt, "capture-hold-" + s, 3, "capture"), 409, "FOLLOW_ACTION_UNAVAILABLE");
        followProjection(receipt, "follow_register", 3, Instant.now(), "RUNNING");
        db.sql("UPDATE control_task_views SET synced_at=now()-interval '11 seconds' WHERE task_ref=?").param("task-" + s).update();
        assertThat(call(cart, "GET", path, null, cartId()).body().path("stale").asBoolean()).isTrue();
        followProjection("different-session", "following", 5, Instant.now(), "RUNNING");
        assertThat(call(cart, "GET", path, null, cartId()).body().path("stale").asBoolean()).isTrue();
    }

    @Test
    void selectedTargetRestoresWithoutRetiredCaptureOrPrivateTargetReference() throws Exception {
        String receipt = appearanceReceipt();
        String expires = Instant.now().plusSeconds(30).toString();
        // MQTT select completion carries only target evidence; the capture is no longer current.
        var registration = Map.of("target_ref", "private-target-ref", "expires_at", expires, "perception_boot_id", "boot-test");
        db.sql("UPDATE control_task_views SET progress_payload=jsonb_set(jsonb_set(progress_payload,'{progress,packages,0,stage}', '\"follow_pending\"'::jsonb),'{progress,packages,0,follow,registration}',?::jsonb),synced_at=now() WHERE task_ref=?")
                .param(json.writeValueAsString(registration)).param("task-" + s).update();
        var response = call(cart, "GET", "/cart-api/follow/" + receipt, null, cartId());
        assertThat(response.status()).isEqualTo(200);
        assertThat(response.body().path("stage").asString()).isEqualTo("follow_pending");
        assertThat(response.body().path("registration")).isEqualTo(json.valueToTree(
                Map.of("registered", true, "expiresAt", expires, "perceptionBootId", "boot-test")));
        assertThat(response.body().toString()).doesNotContain("private-target-ref", "frameRef", "imagePath", "candidates");
    }

    @Test
    void followActionsAreDurableIdempotentAndDoNotPretendRegistrationSucceeded() throws Exception {
        String receipt = appearanceReceipt(), request = "capture-" + s;
        problem(followAction(receipt, request, 2, "capture"), 409, "FOLLOW_REVISION_CHANGED");
        var accepted = followAction(receipt, request, 3, "capture");
        assertThat(accepted.status()).as(accepted.body().toString()).isEqualTo(202);
        assertThat(accepted.body().path("accepted").asBoolean()).isTrue();
        JsonNode message = json.readTree(db.sql("SELECT message::text FROM control_link_outbox WHERE source_ref=?")
                .param("follow:" + request).query(String.class).single());
        assertThat(message.path("type").asString()).isEqualTo("follow.control_requested");
        assertThat(message.path("task_id").asString()).isEqualTo("task-" + s);
        assertThat(message.path("payload").path("robot_id").asString()).isEqualTo(cartId());
        assertThat(message.path("payload").path("receipt_id").asString()).isEqualTo(receipt);
        followProjection(receipt, "follow_registering", 4, Instant.now(), "RUNNING");
        assertThat(followAction(receipt, request, 3, "capture").body()).isEqualTo(accepted.body());
        assertThat(db.sql("SELECT count(*) FROM control_link_outbox WHERE source_ref=?").param("follow:" + request)
                .query(Integer.class).single()).isEqualTo(1);
        problem(followAction(receipt, request, 4, "return"), 409, "IDEMPOTENCY_CONFLICT");
        assertThat(call(cart, "GET", "/cart-api/follow/" + receipt, null, cartId()).body()
                .path("registration").path("registered").asBoolean()).isFalse();
    }

    @Test
    void selectRejectsMixedTargetsUnknownFramesAndClientTargetReference() throws Exception {
        String receipt = appearanceReceipt(), path = "/cart-api/follow/" + receipt + "/actions";
        var body = new java.util.HashMap<String, Object>(Map.of("requestId", "select-" + s, "expectedRevision", 3,
                "operation", "select", "frameRef", "frame-" + s, "candidateId", "person-1"));
        body.put("target_ref", "fabricated");
        problem(call(cart, "POST", path, json.writeValueAsString(body), cartId()), 400, "VALIDATION_FAILED");
        body.remove("target_ref"); body.put("box", List.of(0.1, 0.2, 0.3, 0.4));
        problem(call(cart, "POST", path, json.writeValueAsString(body), cartId()), 400, "VALIDATION_FAILED");
        body.remove("box"); body.put("candidateId", "unknown");
        problem(call(cart, "POST", path, json.writeValueAsString(body), cartId()), 409, "FOLLOW_CANDIDATE_CHANGED");
        body.put("candidateId", "person-1"); body.put("frameRef", "old-frame");
        problem(call(cart, "POST", path, json.writeValueAsString(body), cartId()), 409, "FOLLOW_FRAME_EXPIRED");
        body.put("frameRef", "frame-" + s);
        assertThat(call(cart, "POST", path, json.writeValueAsString(body), cartId()).status()).isEqualTo(202);
        String framePath = "/cart-api/follow/" + receipt + "/frames/frame-" + s;
        problem(call(cart, "GET", framePath, null, cartId()), 503, "FOLLOW_PREVIEW_UNAVAILABLE");
        problem(call(cart, "GET", framePath, null, "other"), 404, "FOLLOW_NOT_FOUND");
        db.sql("UPDATE control_task_views SET progress_payload=jsonb_set(progress_payload,'{progress,packages,0,follow,registration,expires_at}',?::jsonb) WHERE task_ref=?")
                .param(json.writeValueAsString(Instant.now().minusSeconds(1).toString())).param("task-" + s).update();
        body.put("requestId", "expired-select-" + s);
        problem(call(cart, "POST", path, json.writeValueAsString(body), cartId()), 409, "FOLLOW_FRAME_EXPIRED");
        problem(call(cart, "GET", framePath, null, cartId()), 409, "FOLLOW_FRAME_EXPIRED");
    }

    @Test
    void followFrameProxyUsesOnlyConfiguredOriginWithBoundedImagesAndNoRedirects() throws Exception {
        String receipt = appearanceReceipt();
        var server = com.sun.net.httpserver.HttpServer.create(new java.net.InetSocketAddress("127.0.0.1", 0), 0);
        var response = new java.util.concurrent.atomic.AtomicInteger(200);
        var oversized = new java.util.concurrent.atomic.AtomicBoolean();
        var requestedPath = new java.util.concurrent.atomic.AtomicReference<String>();
        var authorization = new java.util.concurrent.atomic.AtomicReference<String>();
        server.createContext("/", request -> {
            requestedPath.set(request.getRequestURI().getPath());
            authorization.set(request.getRequestHeaders().getFirst("Authorization"));
            byte[] bytes = java.util.Base64.getDecoder().decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aEmkAAAAASUVORK5CYII=");
            request.getResponseHeaders().set("Content-Type", "image/png");
            request.getResponseHeaders().set("Location", "http://127.0.0.1:1/forbidden");
            request.sendResponseHeaders(response.get(), oversized.get() ? 2 * 1024 * 1024 + 1 : bytes.length);
            try (var out = request.getResponseBody()) { out.write(bytes); }
            catch (java.io.IOException ignored) { /* Oversized response is intentionally rejected before reading. */ }
        });
        server.start();
        try {
            var service = new FollowService(db, json, exchange, java.time.Clock.systemUTC(), java.time.Duration.ofMinutes(5),
                    "http://127.0.0.1:" + server.getAddress().getPort(), "isolated-preview-token");
            var frame = service.frame(cartId(), receipt, "frame-" + s);
            assertThat(frame.contentType()).isEqualTo("image/png");
            assertThat(requestedPath.get()).isEqualTo("/api/fleet/follow/" + receipt + "/frames/frame-" + s);
            assertThat(authorization.get()).isEqualTo("Bearer isolated-preview-token");
            response.set(302);
            assertThatThrownBy(() -> service.frame(cartId(), receipt, "frame-" + s)).isInstanceOf(ApiException.class);
            response.set(200); oversized.set(true);
            assertThatThrownBy(() -> service.frame(cartId(), receipt, "frame-" + s)).isInstanceOf(ApiException.class);
            assertThatThrownBy(() -> service.frame("other", receipt, "frame-" + s)).isInstanceOf(ApiException.class);
            assertThatThrownBy(() -> new FollowService(db, json, exchange, java.time.Clock.systemUTC(),
                    java.time.Duration.ofMinutes(5), "http://example.invalid", "isolated-preview-token"))
                    .isInstanceOf(IllegalArgumentException.class);
        } finally { server.stop(0); }
    }
}
