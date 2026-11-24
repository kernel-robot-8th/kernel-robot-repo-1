package com.roport.service.api;

import com.roport.service.control.ControlExchange;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URI;
import java.time.Clock;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpStatus;
import org.springframework.jdbc.core.simple.JdbcClient;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import tools.jackson.databind.JsonNode;
import tools.jackson.databind.ObjectMapper;

@Service
class FollowService {
    private static final Set<String> OPERATIONS = Set.of("capture", "select", "release", "pause", "resume", "return");
    private static final int MAX_IMAGE_BYTES = 2 * 1024 * 1024;
    private final JdbcClient db;
    private final ObjectMapper json;
    private final ControlExchange exchange;
    private final Clock clock;
    private final Duration verificationAge;
    private final URI previewBase;
    private final String previewToken;

    FollowService(JdbcClient db, ObjectMapper json, ControlExchange exchange, Clock clock,
            @Value("${service.handover.verification-age:PT5M}") Duration verificationAge,
            @Value("${service.follow.preview-base-url:}") String previewBase,
            @Value("${service.follow.preview-token:${service.control.token:}}") String previewToken) {
        this.db = db; this.json = json; this.exchange = exchange; this.clock = clock;
        this.verificationAge = verificationAge; this.previewToken = previewToken;
        this.previewBase = previewBase.isBlank() ? null : URI.create(previewBase);
        if (this.previewBase != null && (!Set.of("http", "https").contains(this.previewBase.getScheme())
                || this.previewBase.getHost() == null || this.previewBase.getUserInfo() != null
                || this.previewBase.getQuery() != null || this.previewBase.getFragment() != null
                || !Set.of("", "/").contains(this.previewBase.getPath())
                || ("http".equals(this.previewBase.getScheme()) && !Set.of("localhost", "127.0.0.1", "::1", "[::1]")
                        .contains(this.previewBase.getHost())))) {
            throw new IllegalArgumentException("follow preview base requires HTTPS, or loopback HTTP");
        }
    }

    private static ApiException conflict(String code) {
        return new ApiException(HttpStatus.CONFLICT, code, "카트 상태를 새로 확인해 주세요.");
    }

    private static boolean id(String value) {
        return value != null && value.matches("[A-Za-z0-9_.:-]{1,128}") && !Set.of(".", "..").contains(value);
    }

    private static boolean fresh(String value, Instant now, long maxAge) {
        try {
            Instant at = Instant.parse(value);
            return !at.isAfter(now.plusSeconds(1)) && !at.isBefore(now.minusSeconds(maxAge));
        } catch (RuntimeException ex) { return false; }
    }

    private static String timestamp(Object value) {
        return value instanceof java.sql.Timestamp t ? t.toInstant().toString() : "";
    }

    static List<String> supported(JsonNode payload, String cart, Instant now, String syncedAt) {
        if (!fresh(syncedAt, now, 10)) return List.of();
        for (var item : payload.path("progress").path("packages")) {
            if (!cart.equals(item.path("robot").asString()) || !"await_receipt".equals(item.path("stage").asString())
                    || !fresh(item.path("robot_observed_at").asString(), now, 3)) continue;
            var modes = new ArrayList<String>();
            for (var mode : item.path("supported_follow_modes")) {
                String name = mode.asString();
                if (Set.of("marker", "appearance").contains(name) && !modes.contains(name)) modes.add(name);
            }
            return List.copyOf(modes);
        }
        return List.of();
    }

    Map<String, Object> capabilities(String cart, String verification) {
        if (!id(verification)) throw ApiException.badRequest("verificationId가 필요합니다.");
        var rows = db.sql("""
                SELECT v.verified_at,t.progress_payload::text,t.synced_at FROM verifications v
                JOIN pickups p USING(pickup_id) JOIN orders o USING(order_id)
                JOIN control_task_views t USING(pickup_id)
                WHERE v.verification_id=? AND v.actor_ref=? AND v.result='VERIFIED'
                    AND o.business_status='CONFIRMED' AND t.site_id=o.site_id AND t.state='RUNNING'
                """).param(verification).param("cart:" + cart).query().listOfRows();
        if (rows.size() != 1 || !fresh(timestamp(rows.getFirst().get("verified_at")), clock.instant(), verificationAge.toSeconds())) {
            throw conflict("VERIFICATION_REQUIRED");
        }
        var row = rows.getFirst();
        return Map.of("modes", supported(json.readTree(row.get("progress_payload").toString()), cart,
                clock.instant(), timestamp(row.get("synced_at"))));
    }

    private Map<String, Object> owned(String cart, String receipt) {
        if (!id(receipt)) throw ApiException.badRequest("receiptId가 필요합니다.");
        return db.sql("""
                SELECT r.receipt_id,r.pickup_id,r.order_id,r.follow_mode,r.follow_card,a.task_ref,a.cart_ref,o.site_id,
                    v.state,v.progress_payload::text,v.synced_at FROM receipts r
                JOIN handover_approvals a USING(approval_id) JOIN orders o ON o.order_id=r.order_id
                LEFT JOIN control_task_views v ON v.task_ref=a.task_ref
                WHERE r.receipt_id=? AND a.cart_ref=?
                """).param(receipt).param(cart).query().listOfRows().stream().findFirst()
                .orElseThrow(() -> new ApiException(HttpStatus.NOT_FOUND, "FOLLOW_NOT_FOUND", "이 카트의 추종 이용을 찾을 수 없습니다."));
    }

    private JsonNode follow(Map<String, Object> owner, String receipt) {
        if (owner.get("progress_payload") == null) return json.createObjectNode();
        for (var item : json.readTree(owner.get("progress_payload").toString()).path("progress").path("packages")) {
            if (owner.get("cart_ref").equals(item.path("robot").asString())
                    && receipt.equals(item.path("follow").path("session_id").asString())) return item;
        }
        return json.createObjectNode();
    }

    private Map<String, Object> status(Map<String, Object> owner, String receipt) {
        var item = follow(owner, receipt);
        var follow = item.path("follow");
        String updated = timestamp(owner.get("synced_at"));
        boolean stale = !fresh(updated, clock.instant(), 10)
                || !fresh(follow.path("observed_at").asString(), clock.instant(), 3);
        var result = new LinkedHashMap<String, Object>();
        result.put("receiptId", receipt);
        result.put("mode", HandoverService.followMode((String) owner.get("follow_mode"), (Integer) owner.get("follow_card")));
        result.put("stage", item.path("stage").asString("follow_pending"));
        result.put("status", owner.get("state") == null ? "PENDING" : owner.get("state"));
        result.put("reason", follow.path("reason").asString(item.path("reason").asString("")));
        result.put("updatedAt", updated.isEmpty() ? null : updated);
        result.put("revision", follow.path("revision").asLong(0));
        var actions = new ArrayList<String>();
        if (!stale && "RUNNING".equals(owner.get("state"))) {
            for (var op : follow.path("allowed_actions")) if (OPERATIONS.contains(op.asString())) actions.add(op.asString());
        }
        result.put("allowedActions", List.copyOf(actions));
        result.put("stale", stale);
        var registration = follow.path("registration");
        if (registration.isObject()) {
            var mapped = new LinkedHashMap<String, Object>();
            boolean registered = registration.path("target_ref").isString() && !registration.path("target_ref").asString().isBlank();
            mapped.put("expiresAt", registration.path("expires_at").asString());
            mapped.put("perceptionBootId", registration.path("perception_boot_id").asString());
            mapped.put("registered", registered);
            if (!registered) {
                mapped.put("frameRef", registration.path("frame_ref").asString());
                mapped.put("imagePath", "/cart-api/follow/" + receipt + "/frames/" + registration.path("frame_ref").asString());
                var candidates = new ArrayList<Map<String, Object>>();
                for (var candidate : registration.path("candidates")) {
                    candidates.add(Map.of("candidateId", candidate.path("candidate_id").asString(), "box", candidate.path("box")));
                }
                mapped.put("candidates", candidates);
            }
            result.put("registration", mapped);
        }
        var last = follow.path("last_request");
        if (last.isObject()) result.put("lastRequest", Map.of("requestId", last.path("request_id").asString(),
                "status", last.path("status").asString(), "reason", last.path("reason").asString("")));
        return result;
    }

    public Map<String, Object> status(String cart, String receipt) { return status(owned(cart, receipt), receipt); }

    @Transactional
    public Map<String, Object> action(String cart, String receipt, JsonNode body) {
        if (body == null || !body.isObject()) throw ApiException.badRequest("추종 요청이 필요합니다.");
        String request = body.path("requestId").asString(), operation = body.path("operation").asString();
        if (!body.path("requestId").isString() || !body.path("operation").isString()
                || !id(request) || !OPERATIONS.contains(operation) || !body.path("expectedRevision").isIntegralNumber()
                || !body.path("expectedRevision").canConvertToLong() || body.path("expectedRevision").asLong() < 0
                || body.path("expectedRevision").asLong() > 9007199254740991L) {
            throw ApiException.badRequest("requestId·expectedRevision·operation을 확인해 주세요.");
        }
        var fields = new HashSet<>(Set.of("requestId", "expectedRevision", "operation"));
        if (operation.equals("select")) {
            fields.add("frameRef");
            if (body.has("candidateId") == body.has("box")) throw ApiException.badRequest("후보 하나 또는 선택 영역 하나가 필요합니다.");
            fields.add(body.has("candidateId") ? "candidateId" : "box");
            if (!body.path("frameRef").isString() || !id(body.path("frameRef").asString())) throw ApiException.badRequest("frameRef가 필요합니다.");
            if (body.has("candidateId") && (!body.path("candidateId").isString() || !id(body.path("candidateId").asString())))
                throw ApiException.badRequest("candidateId를 확인해 주세요.");
            if (body.has("box")) validateBox(body.path("box"));
        }
        if (!body.propertyNames().equals(fields)) throw ApiException.badRequest("지원하지 않는 추종 요청 필드입니다.");
        // Serializes duplicate request IDs before rechecking the current projection.
        db.sql("SELECT pg_advisory_xact_lock(71519007)").query().singleValue();
        var owner = owned(cart, receipt);
        var payload = new LinkedHashMap<String, Object>();
        payload.put("receipt_id", receipt); payload.put("reservation_id", owner.get("pickup_id"));
        payload.put("order_id", owner.get("order_id")); payload.put("robot_id", cart);
        payload.put("request_id", request); payload.put("expected_revision", body.path("expectedRevision").asLong());
        payload.put("operation", operation);
        if (operation.equals("select")) {
            payload.put("frame_ref", body.path("frameRef").asString());
            if (body.has("candidateId")) payload.put("candidate_id", body.path("candidateId").asString());
            else payload.put("box", body.path("box"));
        }
        var prior = db.sql("SELECT message->'payload' = ?::jsonb FROM control_link_outbox WHERE source_ref=?")
                .param(json.writeValueAsString(payload)).param("follow:" + request).query(Boolean.class).optional();
        if (prior.isPresent()) {
            if (!prior.get()) throw conflict("IDEMPOTENCY_CONFLICT");
            return Map.of("accepted", true, "requestId", request);
        }
        var status = status(owner, receipt);
        if (!java.util.Objects.equals(status.get("revision"), body.path("expectedRevision").asLong())) throw conflict("FOLLOW_REVISION_CHANGED");
        if (!((List<?>) status.get("allowedActions")).contains(operation)) throw conflict("FOLLOW_ACTION_UNAVAILABLE");
        if (operation.equals("select")) {
            checkFrame(owner, receipt, body.path("frameRef").asString());
            if (body.has("candidateId")) {
                boolean known = false;
                for (var candidate : follow(owner, receipt).path("follow").path("registration").path("candidates")) {
                    if (body.path("candidateId").equals(candidate.path("candidate_id"))) known = true;
                }
                if (!known) throw conflict("FOLLOW_CANDIDATE_CHANGED");
            }
        }
        var message = exchange.envelope(owner.get("site_id").toString(), "follow.control_requested", payload);
        message.put("task_id", owner.get("task_ref"));
        db.sql("INSERT INTO control_link_outbox(message_id,site_id,source_ref,message) VALUES(?,?,?,?::jsonb)")
                .param(message.get("message_id")).param(owner.get("site_id")).param("follow:" + request)
                .param(json.writeValueAsString(message)).update();
        return Map.of("accepted", true, "requestId", request);
    }

    private static void validateBox(JsonNode box) {
        if (!box.isArray() || box.size() != 4) throw ApiException.badRequest("선택 영역은 4개 좌표입니다.");
        for (var value : box) if (!value.isNumber() || !Double.isFinite(value.asDouble()) || value.asDouble() < 0 || value.asDouble() > 1)
            throw ApiException.badRequest("선택 좌표는 0~1 범위입니다.");
        if (box.get(0).asDouble() >= box.get(2).asDouble() || box.get(1).asDouble() >= box.get(3).asDouble())
            throw ApiException.badRequest("선택 영역의 폭과 높이가 필요합니다.");
    }

    private void checkFrame(Map<String, Object> owner, String receipt, String frame) {
        var registration = follow(owner, receipt).path("follow").path("registration");
        try {
            if (id(frame) && frame.equals(registration.path("frame_ref").asString())
                    && Instant.parse(registration.path("expires_at").asString()).isAfter(clock.instant())) return;
        } catch (RuntimeException ignored) { /* Reject invalid or absent expiry. */ }
        throw conflict("FOLLOW_FRAME_EXPIRED");
    }

    record Frame(byte[] bytes, String contentType) {}

    Frame frame(String cart, String receipt, String frame) {
        var owner = owned(cart, receipt);
        checkFrame(owner, receipt, frame);
        if (Boolean.TRUE.equals(status(owner, receipt).get("stale"))) throw conflict("FOLLOW_STATE_STALE");
        if (previewBase == null || previewToken.isBlank()) throw new ApiException(HttpStatus.SERVICE_UNAVAILABLE,
                "FOLLOW_PREVIEW_UNAVAILABLE", "카트 영상 연결을 기다리고 있습니다.");
        HttpURLConnection connection = null;
        try {
            URI url = previewBase.resolve("/api/fleet/follow/" + receipt + "/frames/" + frame);
            connection = (HttpURLConnection) url.toURL().openConnection();
            connection.setInstanceFollowRedirects(false);
            connection.setConnectTimeout(1500); connection.setReadTimeout(2500);
            connection.setRequestProperty("Authorization", "Bearer " + previewToken);
            connection.setRequestProperty("Accept", "image/jpeg,image/png");
            String type = connection.getContentType();
            type = type == null ? "" : type.split(";", 2)[0].trim();
            if (connection.getResponseCode() != 200 || !Set.of("image/jpeg", "image/png").contains(type)
                    || connection.getContentLengthLong() > MAX_IMAGE_BYTES) throw new IOException("invalid frame response");
            try (var stream = connection.getInputStream()) {
                byte[] data = stream.readNBytes(MAX_IMAGE_BYTES + 1);
                if (data.length == 0 || data.length > MAX_IMAGE_BYTES) throw new IOException("invalid frame size");
                return new Frame(data, type);
            }
        } catch (IOException | IllegalArgumentException ex) {
            throw new ApiException(HttpStatus.SERVICE_UNAVAILABLE, "FOLLOW_PREVIEW_UNAVAILABLE", "카트 영상을 다시 촬영해 주세요.");
        } finally { if (connection != null) connection.disconnect(); }
    }
}
