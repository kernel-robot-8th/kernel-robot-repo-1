package com.roport.service.api;

import static org.assertj.core.api.Assertions.assertThat;

import java.net.CookieManager;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.sql.Connection;
import java.sql.PreparedStatement;
import java.time.Clock;
import java.time.Instant;
import java.time.ZoneOffset;
import java.util.List;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import javax.sql.DataSource;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.context.TestConfiguration;
import org.springframework.boot.test.web.server.LocalServerPort;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Primary;
import org.springframework.jdbc.core.simple.JdbcClient;
import tools.jackson.databind.JsonNode;
import tools.jackson.databind.ObjectMapper;

/** 실제 PostgreSQL(SERVICE_DB_URL)의 격리 테스트 DB에 가짜 데이터를 넣고 HTTP로 검사한다. 행은 실행마다 고유 ID를 쓴다. */
@SpringBootTest(webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT)
class CustomerApiTest {
    static final Instant NOW = Instant.parse("2030-01-01T00:10:00Z");

    @TestConfiguration
    static class FixedClock {
        @Bean
        @Primary
        Clock fixedClock() {
            return Clock.fixed(NOW, ZoneOffset.UTC);
        }
    }

    @LocalServerPort
    int port;
    @Autowired
    JdbcClient db;
    @Autowired
    ObjectMapper json;
    @Autowired
    DataSource dataSource;

    String s; // 실행별 접미사
    HttpClient alice;
    HttpClient bob;

    record Res(int status, String contentType, String setCookie, JsonNode body) {}

    @BeforeEach
    void seed() {
        // 기본 SERVICE_DB_URL은 개발 DB(roport_service)다. 가짜 행을 남기므로 _test DB에서만 돈다.
        assertThat(db.sql("SELECT current_database()").query(String.class).single())
                .as("SERVICE_DB_URL은 이름이 _test로 끝나는 격리 DB여야 한다").endsWith("_test");
        s = UUID.randomUUID().toString().substring(0, 8);
        alice = HttpClient.newBuilder().cookieHandler(new CookieManager()).build();
        bob = HttpClient.newBuilder().cookieHandler(new CookieManager()).build();
        db.sql("INSERT INTO members (member_id, login_id, display_name) VALUES (?, ?, 'A'), (?, ?, 'B')")
                .param("ma" + s).param("alice" + s).param("mb" + s).param("bob" + s).update();
        db.sql("""
                INSERT INTO orders (order_id, order_no, member_id, site_id, revision, business_status, ordered_at) VALUES
                (?, ?, ?, 'DEV-SITE', 0, 'CONFIRMED', '2029-12-31T00:00Z'),
                (?, ?, ?, 'DEV-SITE', 0, 'RECEIVED', '2029-12-30T00:00Z'),
                (?, ?, ?, 'DEV-SITE', 0, 'CONFIRMED', '2029-12-31T00:00Z')""")
                .param("oa" + s).param("NA" + s).param("ma" + s)
                .param("or" + s).param("NR" + s).param("ma" + s)
                .param("ob" + s).param("NB" + s).param("mb" + s).update();
        db.sql("INSERT INTO order_items (item_id, order_id, product_name, quantity) VALUES (?, ?, 'test-item', 2)")
                .param("ia" + s).param("oa" + s).update();
        db.sql("""
                INSERT INTO pickup_locations (location_id, site_id, name, control_point_ref, is_active) VALUES
                (?, 'DEV-SITE', 'test-gate', ?, true), (?, 'DEV-SITE', 'test-off', ?, false),
                (?, 'OTHER-SITE', 'test-other-site', ?, true)""")
                .param("la" + s).param("DEV-a" + s).param("lx" + s).param("DEV-x" + s)
                .param("lo" + s).param("DEV-o" + s).update();
        db.sql("""
                INSERT INTO flights (flight_id, flight_no, service_date, boarding_start, boarding_end, departure_at, gate_code, revision)
                VALUES (?, ?, '2030-01-01', '2030-01-01T03:00Z', '2030-01-01T03:30Z', '2030-01-01T03:50Z', 'G1', 3)""")
                .param("fa" + s).param("T" + s).update();
    }

    Res call(HttpClient c, String method, String path, String body) throws Exception {
        return call(c, method, path, body, "application/json");
    }

    Res call(HttpClient c, String method, String path, String body, String contentType) throws Exception {
        HttpRequest.Builder b = HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + path))
                .header("Content-Type", contentType)
                .method(method, body == null ? HttpRequest.BodyPublishers.noBody() : HttpRequest.BodyPublishers.ofString(body));
        HttpResponse<String> r = c.send(b.build(), HttpResponse.BodyHandlers.ofString());
        return toRes(r);
    }

    Res toRes(HttpResponse<String> r) {
        return new Res(r.statusCode(), r.headers().firstValue("Content-Type").orElse(""),
                r.headers().firstValue("Set-Cookie").orElse(""),
                r.body().isEmpty() ? null : json.readTree(r.body()));
    }

    void login(HttpClient c, String loginId) throws Exception {
        assertThat(call(c, "POST", "/api/session", "{\"loginId\":\"" + loginId + "\"}").status()).isEqualTo(200);
    }

    static void problem(Res r, int status, String code) {
        frameworkProblem(r, status);
        assertThat(r.body().path("code").asString("")).as(String.valueOf(r.body())).isEqualTo(code);
    }

    /** 프레임워크 기본 오류도 problem+json {type,title,status}. code는 계약 미정이라 검사하지 않는다. */
    static void frameworkProblem(Res r, int status) {
        assertThat(r.status()).as(String.valueOf(r.body())).isEqualTo(status);
        assertThat(r.contentType()).as(String.valueOf(r.body())).startsWith("application/problem+json");
        assertThat(r.body().path("status").asInt()).as(String.valueOf(r.body())).isEqualTo(status);
        assertThat(r.body().path("type").asString("")).as(String.valueOf(r.body())).isEqualTo("about:blank");
        assertThat(r.body().path("title").asString("")).as(String.valueOf(r.body())).isNotBlank();
    }

    String pickupBody(String requestId, String orderId, String pickupAt) {
        return """
                {"requestId":"%s","orderId":"%s","locationId":"la%s","flightId":"fa%s","pickupAt":"%s","extra":1}"""
                .formatted(requestId, orderId, s, s, pickupAt);
    }

    @Test
    void sessionLifecycle() throws Exception {
        problem(call(alice, "GET", "/api/orders", null), 401, "UNAUTHENTICATED");
        problem(call(alice, "GET", "/api/session", null), 401, "UNAUTHENTICATED");
        problem(call(alice, "DELETE", "/api/session", null), 401, "UNAUTHENTICATED");
        // 세션 예외는 POST /api/session 하나뿐이다. 다른 POST는 본문 검증보다 먼저 401.
        problem(call(alice, "POST", "/api/pickups", pickupBody("rqu" + s, "oa" + s, "2030-01-01T01:30:00Z")),
                401, "UNAUTHENTICATED");
        problem(call(alice, "POST", "/api/session", "{\"loginId\":\"\"}"), 400, "VALIDATION_FAILED");
        problem(call(alice, "POST", "/api/session", "not-json"), 400, "VALIDATION_FAILED");
        problem(call(alice, "POST", "/api/session", "{\"loginId\":\"nobody" + s + "\"}"), 401, "LOGIN_FAILED");

        Res r = call(alice, "POST", "/api/session", "{\"loginId\":\"alice" + s + "\"}");
        assertThat(r.status()).isEqualTo(200);
        assertThat(r.body().get("memberId").asString()).isEqualTo("ma" + s);
        assertThat(r.setCookie()).contains("JSESSIONID").containsIgnoringCase("HttpOnly").contains("SameSite=Strict");
        assertThat(call(alice, "GET", "/api/session", null).body().get("displayName").asString()).isEqualTo("A");

        assertThat(call(alice, "DELETE", "/api/session", null).status()).isEqualTo(204);
        problem(call(alice, "GET", "/api/session", null), 401, "UNAUTHENTICATED");
    }

    static String sessionId(String setCookie) {
        java.util.regex.Matcher m = java.util.regex.Pattern.compile("JSESSIONID=([^;]+)").matcher(setCookie);
        assertThat(m.find()).as("Set-Cookie: " + setCookie).isTrue();
        return m.group(1);
    }

    @Test
    void reloginReplacesSession() throws Exception {
        // 세션 고정 방지: 로그인하면 기존 세션 ID는 폐기되고 새 ID가 발급된다.
        String before = sessionId(call(alice, "POST", "/api/session", "{\"loginId\":\"bob" + s + "\"}").setCookie());
        String after = sessionId(call(alice, "POST", "/api/session", "{\"loginId\":\"alice" + s + "\"}").setCookie());
        assertThat(after).isNotEqualTo(before);
        HttpResponse<String> old = HttpClient.newHttpClient().send(
                HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + "/api/session"))
                        .header("Cookie", "JSESSIONID=" + before).GET().build(),
                HttpResponse.BodyHandlers.ofString());
        problem(toRes(old), 401, "UNAUTHENTICATED");
    }

    @Test
    void frameworkErrorsAreProblemJson() throws Exception {
        frameworkProblem(call(alice, "PUT", "/api/session", "{}"), 405);
        frameworkProblem(call(alice, "POST", "/api/session", "loginId=x", "text/plain"), 415);
        login(alice, "alice" + s);
        frameworkProblem(call(alice, "GET", "/api/no-such-path", null), 404);
    }

    @Test
    void queries() throws Exception {
        login(alice, "alice" + s);
        JsonNode orders = call(alice, "GET", "/api/orders", null).body();
        assertThat(orders.size()).isEqualTo(1); // CONFIRMED만, 다른 회원 주문 제외
        assertThat(orders.get(0).get("orderId").asString()).isEqualTo("oa" + s);
        assertThat(orders.get(0).get("orderedAt").asString()).isEqualTo("2029-12-31T00:00:00Z");
        assertThat(orders.get(0).get("items").get(0).get("quantity").asInt()).isEqualTo(2);
        assertThat(orders.get(0).get("activePickup").isNull()).isTrue();

        JsonNode locations = call(alice, "GET", "/api/pickup-locations", null).body();
        assertThat(locations.findValuesAsString("locationId")).contains("la" + s).doesNotContain("lx" + s);

        JsonNode flights = call(alice, "GET", "/api/flights?serviceDate=2030-01-01", null).body();
        JsonNode f = flights.valueStream().filter(n -> n.get("flightId").asString().equals("fa" + s)).findFirst().orElseThrow();
        assertThat(f.get("serviceDate").asString()).isEqualTo("2030-01-01");
        assertThat(f.get("boardingStart").asString()).isEqualTo("2030-01-01T03:00:00Z");
        assertThat(call(alice, "GET", "/api/flights?serviceDate=2029-01-01", null).body().findValuesAsString("flightId"))
                .doesNotContain("fa" + s);
        problem(call(alice, "GET", "/api/flights?serviceDate=2030-1-1", null), 400, "VALIDATION_FAILED");

        String q = "?flightId=fa" + s + "&locationId=la" + s;
        JsonNode times = call(alice, "GET", "/api/orders/oa" + s + "/pickup-times" + q, null).body().get("times");
        assertThat(times.valueStream().map(JsonNode::asString).toList())
                .containsExactly("2030-01-01T01:30:00Z", "2030-01-01T02:00:00Z", "2030-01-01T02:30:00Z");
        problem(call(alice, "GET", "/api/orders/ob" + s + "/pickup-times" + q, null), 404, "ORDER_NOT_FOUND");
        problem(call(alice, "GET", "/api/orders/oa" + s + "/pickup-times?flightId=fa" + s + "&locationId=lx" + s, null),
                404, "LOCATION_NOT_FOUND");
        // 활성이어도 주문과 다른 site의 위치는 없는 위치로 본다.
        problem(call(alice, "GET", "/api/orders/oa" + s + "/pickup-times?flightId=fa" + s + "&locationId=lo" + s, null),
                404, "LOCATION_NOT_FOUND");
        problem(call(alice, "GET", "/api/orders/oa" + s + "/pickup-times?flightId=nope&locationId=la" + s, null),
                404, "FLIGHT_NOT_FOUND");
        problem(call(alice, "GET", "/api/orders/oa" + s + "/pickup-times", null), 400, "VALIDATION_FAILED");
    }

    @Test
    void createPickup() throws Exception {
        login(alice, "alice" + s);
        login(bob, "bob" + s);
        String rq = "rq" + s;

        problem(call(alice, "POST", "/api/pickups", pickupBody(rq, "oa" + s, "2030-01-01T01:30:00")), 400, "VALIDATION_FAILED");
        problem(call(alice, "POST", "/api/pickups", "{\"requestId\":\"x\"}"), 400, "VALIDATION_FAILED");
        problem(call(alice, "POST", "/api/pickups", pickupBody(rq, "oa" + s, "2030-01-01T01:45:00Z")), 422, "PICKUP_TIME_NOT_AVAILABLE");
        problem(call(alice, "POST", "/api/pickups", pickupBody(rq, "or" + s, "2030-01-01T01:30:00Z")), 409, "ORDER_NOT_RESERVABLE");
        problem(call(alice, "POST", "/api/pickups", pickupBody(rq, "ob" + s, "2030-01-01T01:30:00Z")), 404, "ORDER_NOT_FOUND");
        problem(call(alice, "POST", "/api/pickups", pickupBody(rq, "oa" + s, "2030-01-01T01:30:00Z").replace("la" + s, "lx" + s)),
                404, "LOCATION_NOT_FOUND");
        problem(call(alice, "POST", "/api/pickups", pickupBody(rq, "oa" + s, "2030-01-01T01:30:00Z").replace("la" + s, "lo" + s)),
                404, "LOCATION_NOT_FOUND");
        problem(call(alice, "POST", "/api/pickups", pickupBody(rq, "oa" + s, "2030-01-01T01:30:00Z").replace("fa" + s, "nope" + s)),
                404, "FLIGHT_NOT_FOUND");

        // 실패한 요청은 저장되지 않아 같은 requestId로 성공할 수 있다. offset +09:00 = 01:30Z
        Res first = call(alice, "POST", "/api/pickups", pickupBody(rq, "oa" + s, "2030-01-01T10:30:00+09:00"));
        assertThat(first.status()).as(String.valueOf(first.body())).isEqualTo(201);
        JsonNode p = first.body();
        assertThat(p.get("status").asString()).isEqualTo("REQUESTED");
        assertThat(p.get("revision").asLong()).isZero();
        assertThat(p.get("pickupAt").asString()).isEqualTo("2030-01-01T01:30:00Z");
        assertThat(p.get("reservationNo").asString()).matches("[2-9A-HJ-NP-Z]{8}");
        assertThat(p.get("orderNo").asString()).isEqualTo("NA" + s);
        assertThat(p.get("location").get("locationId").asString()).isEqualTo("la" + s);
        assertThat(p.get("flight").get("revision").asLong()).isEqualTo(3);
        assertThat(p.get("flight").get("boardingStart").asString()).isEqualTo("2030-01-01T03:00:00Z");
        assertThat(db.sql("SELECT channel FROM pickups WHERE request_id = ?").param(rq).query(String.class).single())
                .isEqualTo("WEB");

        Res again = call(alice, "POST", "/api/pickups", pickupBody(rq, "oa" + s, "2030-01-01T01:30:00Z"));
        assertThat(again.status()).isEqualTo(200);
        assertThat(again.body()).isEqualTo(p);
        problem(call(alice, "POST", "/api/pickups", pickupBody(rq, "oa" + s, "2030-01-01T02:00:00Z")), 409, "IDEMPOTENCY_CONFLICT");
        problem(call(bob, "POST", "/api/pickups", pickupBody(rq, "ob" + s, "2030-01-01T01:30:00Z")), 409, "IDEMPOTENCY_CONFLICT");
        // 다른 회원이 본문까지 똑같이 보내도 저장본을 돌려주지 않는다.
        problem(call(bob, "POST", "/api/pickups", pickupBody(rq, "oa" + s, "2030-01-01T01:30:00Z")), 409, "IDEMPOTENCY_CONFLICT");
        // 필드 하나만 달라도 충돌(검증보다 재전송 판정이 먼저다).
        problem(call(alice, "POST", "/api/pickups", pickupBody(rq, "oa" + s, "2030-01-01T01:30:00Z").replace("la" + s, "lx" + s)),
                409, "IDEMPOTENCY_CONFLICT");
        problem(call(alice, "POST", "/api/pickups", pickupBody(rq, "oa" + s, "2030-01-01T01:30:00Z").replace("fa" + s, "nope" + s)),
                409, "IDEMPOTENCY_CONFLICT");
        problem(call(alice, "POST", "/api/pickups", pickupBody("rq2" + s, "oa" + s, "2030-01-01T02:00:00Z")), 409, "ACTIVE_PICKUP_EXISTS");

        assertThat(call(alice, "GET", "/api/orders", null).body().get(0).get("activePickup").get("pickupId"))
                .isEqualTo(p.get("pickupId"));
        assertThat(call(alice, "GET", "/api/pickups", null).body()).containsExactly(p);
        assertThat(call(alice, "GET", "/api/pickups/" + p.get("pickupId").asString(), null).body()).isEqualTo(p);
        problem(call(bob, "GET", "/api/pickups/" + p.get("pickupId").asString(), null), 404, "PICKUP_NOT_FOUND");
        assertThat(call(bob, "GET", "/api/pickups", null).body()).isEmpty();
        problem(call(HttpClient.newHttpClient(), "GET", "/api/pickups/" + p.get("pickupId").asString(), null),
                401, "UNAUTHENTICATED");
    }

    @Test
    void reserveAgainAfterInactivePickup() throws Exception {
        // 활성 집합(pickups_one_active_per_order) 밖 상태는 재예약을 막지 않는다.
        db.sql("""
                INSERT INTO pickups (pickup_id, request_id, reservation_no, order_id, location_id, flight_id,
                    flight_revision, flight_snapshot, pickup_at, channel, status, revision)
                VALUES (?, ?, ?, ?, ?, ?, 3, '{}', '2030-01-01T01:30Z', 'WEB', 'CANCELLED', 1)""")
                .param("pc" + s).param("rqx" + s).param("X" + s).param("oa" + s).param("la" + s).param("fa" + s).update();
        login(alice, "alice" + s);
        assertThat(call(alice, "GET", "/api/orders", null).body().get(0).get("activePickup").isNull()).isTrue();
        Res r = call(alice, "POST", "/api/pickups", pickupBody("rqn" + s, "oa" + s, "2030-01-01T02:00:00Z"));
        assertThat(r.status()).as(String.valueOf(r.body())).isEqualTo(201);
        assertThat(call(alice, "GET", "/api/pickups", null).body().size()).isEqualTo(2);
    }

    @Test
    void candidateTimesBoundary() {
        Instant boarding = Instant.parse("2030-01-01T03:00:00Z");
        // 현재+60분이 정확히 슬롯이면 포함, 1ns라도 지나면 다음 슬롯. boarding_start 자체는 제외.
        assertThat(new PickupService(null, Clock.fixed(Instant.parse("2030-01-01T00:30:00Z"), ZoneOffset.UTC), null)
                .candidateTimes(boarding)).first().isEqualTo(Instant.parse("2030-01-01T01:30:00Z"));
        assertThat(new PickupService(null, Clock.fixed(Instant.parse("2030-01-01T00:30:00.000000001Z"), ZoneOffset.UTC), null)
                .candidateTimes(boarding)).first().isEqualTo(Instant.parse("2030-01-01T02:00:00Z"));
        assertThat(new PickupService(null, Clock.fixed(Instant.parse("2030-01-01T02:00:00Z"), ZoneOffset.UTC), null)
                .candidateTimes(boarding)).isEmpty();
    }

    @Test
    void reservationWaitsForInFlightOrderUpdate() throws Exception {
        // 계약: orders FOR UPDATE. 진행 중인 주문 갱신(취소)이 커밋될 때까지 기다리고 그 결과로 판정한다.
        // FK 검사(FOR KEY SHARE)는 UPDATE의 NO KEY UPDATE 잠금과 충돌하지 않으므로 잠금이 없으면 201이 먼저 나온다.
        login(alice, "alice" + s);
        try (Connection c = dataSource.getConnection()) {
            c.setAutoCommit(false);
            try (PreparedStatement st = c.prepareStatement("UPDATE orders SET business_status = 'CANCELLED' WHERE order_id = ?")) {
                st.setString(1, "oa" + s);
                st.executeUpdate();
            }
            CompletableFuture<HttpResponse<String>> sent = alice.sendAsync(
                    HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + "/api/pickups"))
                            .header("Content-Type", "application/json")
                            .POST(HttpRequest.BodyPublishers.ofString(pickupBody("rql" + s, "oa" + s, "2030-01-01T01:30:00Z")))
                            .build(),
                    HttpResponse.BodyHandlers.ofString());
            long deadline = System.nanoTime() + java.time.Duration.ofSeconds(5).toNanos();
            while (!sent.isDone() && System.nanoTime() < deadline && !waitingOnLock()) {
                Thread.sleep(20);
            }
            c.commit();
            problem(toRes(sent.join()), 409, "ORDER_NOT_RESERVABLE");
        }
    }

    boolean waitingOnLock() {
        return db.sql("SELECT count(*) FROM pg_stat_activity WHERE datname = current_database() AND wait_event_type = 'Lock'")
                .query(Long.class).single() > 0;
    }

    @Test
    void concurrentRetryCreatesOnce() throws Exception {
        login(bob, "bob" + s);
        HttpRequest req = HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + "/api/pickups"))
                .header("Content-Type", "application/json")
                .POST(HttpRequest.BodyPublishers.ofString(pickupBody("rqc" + s, "ob" + s, "2030-01-01T02:30:00Z")))
                .build();
        List<CompletableFuture<HttpResponse<String>>> sent = java.util.stream.IntStream.range(0, 4)
                .mapToObj(i -> bob.sendAsync(req, HttpResponse.BodyHandlers.ofString())).toList();
        List<Res> results = sent.stream().map(CompletableFuture::join).map(this::toRes).toList();
        assertThat(results).extracting(Res::status).containsOnly(201, 200).filteredOn(st -> st == 201).hasSize(1);
        assertThat(results).extracting(Res::body).containsOnly(results.get(0).body());
        assertThat(db.sql("SELECT count(*) FROM pickups WHERE order_id = ?").param("ob" + s).query(Long.class).single())
                .isEqualTo(1);
    }
}
