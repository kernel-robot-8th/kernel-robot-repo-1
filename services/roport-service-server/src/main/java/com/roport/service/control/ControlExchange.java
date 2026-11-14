package com.roport.service.control;

import java.time.Instant;
import java.time.OffsetDateTime;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.UUID;
import org.springframework.jdbc.core.simple.JdbcClient;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import tools.jackson.databind.JsonNode;
import tools.jackson.databind.ObjectMapper;

/** Durable service transport. Robot completion never creates a customer receipt. */
@Service
public class ControlExchange {
    private final JdbcClient db;
    private final ObjectMapper json;

    public ControlExchange(JdbcClient db, ObjectMapper json) {
        this.db = db;
        this.json = json;
    }

    public Map<String, Object> envelope(String site, String type, Map<String, Object> payload) {
        Map<String, Object> message = new LinkedHashMap<>();
        message.put("schema_version", "1.0");
        message.put("message_id", UUID.randomUUID().toString());
        message.put("type", type);
        message.put("site_id", site);
        message.put("source", "service-01");
        message.put("target", "control-01");
        message.put("sent_at", Instant.now().toString());
        message.put("payload", payload);
        return message;
    }

    private void enqueue(String source, Map<String, Object> message) {
        db.sql("INSERT INTO control_link_outbox(message_id,site_id,source_ref,message) VALUES (?,?,?,?::jsonb) ON CONFLICT(source_ref) DO NOTHING")
                .param(message.get("message_id")).param(message.get("site_id")).param(source)
                .param(json.writeValueAsString(message)).update();
    }

    /** Called inside reservation creation's transaction, including its package snapshot. */
    @Transactional
    public void reservationCreated(String pickupId) {
        var row = db.sql("""
                SELECT p.pickup_id,p.order_id,o.site_id,o.revision,l.control_point_ref,
                    to_char(p.pickup_at AT TIME ZONE 'UTC', 'YYYY-MM-DD"T"HH24:MI:SS.US"Z"') AS pickup_at
                FROM pickups p JOIN orders o USING(order_id) JOIN pickup_locations l USING(location_id)
                WHERE pickup_id=?""").param(pickupId).query().singleRow();
        List<Map<String, Object>> packages = db.sql("""
                SELECT package_id,1 AS quantity FROM packages
                WHERE order_id=? AND inactive_at IS NULL AND manifest_locked_at IS NOT NULL ORDER BY package_id""")
                .param(row.get("order_id")).query().listOfRows();
        var payload = new LinkedHashMap<String, Object>();
        payload.put("order_id", row.get("order_id"));
        payload.put("reservation_id", pickupId);
        payload.put("operation", "customer_service");
        payload.put("packages", packages);
        payload.put("pickup_point_id", row.get("control_point_ref"));
        payload.put("pickup_at", row.get("pickup_at").toString());
        payload.put("order_revision", row.get("revision"));
        enqueue("reservation:" + pickupId, envelope((String) row.get("site_id"), "task.requested", payload));
    }

    /** Existing receipt/approval workflow owns these rows; this link only forwards committed evidence. */
    @Transactional
    public void publishReceipts(String site) {
        var rows = db.sql("""
                SELECT r.receipt_id,r.pickup_id,r.order_id,v.task_ref
                FROM receipts r JOIN control_task_views v USING(pickup_id)
                WHERE v.site_id=? AND NOT EXISTS
                (SELECT 1 FROM control_link_outbox b WHERE b.source_ref='receipt:' || r.receipt_id)""")
                .param(site).query().listOfRows();
        for (var row : rows) {
            var packages = db.sql("SELECT package_id FROM receipt_packages WHERE receipt_id=? ORDER BY package_id")
                    .param(row.get("receipt_id")).query(String.class).list();
            if (packages.isEmpty()) continue; // Receipt alone is not evidence for an unspecified package.
            var message = envelope(site, "handover.receipt_recorded", Map.of(
                    "receipt_id", row.get("receipt_id"), "reservation_id", row.get("pickup_id"),
                    "order_id", row.get("order_id"), "package_ids", packages));
            message.put("task_id", row.get("task_ref"));
            enqueue("receipt:" + row.get("receipt_id"), message);
        }
    }

    public List<String> pending(String site) {
        return db.sql("SELECT message::text FROM control_link_outbox WHERE site_id=? AND acknowledged_at IS NULL ORDER BY created_at,message_id")
                .param(site).query(String.class).list();
    }

    private static String text(JsonNode node, String key) {
        var value = node.get(key);
        if (value == null || !value.isString() || value.asString().isBlank()) {
            throw new IllegalArgumentException("missing " + key);
        }
        return value.asString();
    }

    /** Return value is sent only after the @Transactional interceptor commits. */
    @Transactional
    public Map<String, Object> receive(String site, String raw) {
        JsonNode message = json.readTree(raw);
        if (!site.equals(text(message, "site_id"))) throw new IllegalArgumentException("site mismatch");
        String type = text(message, "type");
        JsonNode payload = message.path("payload");
        if (type.equals("message.ack")) {
            db.sql("UPDATE control_link_outbox SET acknowledged_at=now() WHERE site_id=? AND message_id=? AND acknowledged_at IS NULL")
                    .param(site).param(text(payload, "message_id")).update();
            return null;
        }
        if (!Set.of("task.accepted", "task.progress", "task.completed", "task.failed", "task.rejected").contains(type)) {
            throw new IllegalArgumentException("unsupported control event");
        }
        String id = text(message, "message_id");
        // Lock transport receipt and view update together for duplicate sessions/replays.
        db.sql("SELECT pg_advisory_xact_lock(71519005)").query().singleValue();
        var previous = db.sql("SELECT message::text FROM control_link_inbox WHERE message_id=?")
                .param(id).query(String.class).optional();
        if (previous.isPresent()) {
            if (!json.readTree(previous.get()).equals(message)) throw new IllegalArgumentException("message_id reused");
            return envelope(site, "message.ack", Map.of("message_id", id));
        }
        String task = text(message, "task_id");
        String pickup = text(payload, "reservation_id");
        String order = text(payload, "order_id");
        String state = text(payload, "state");
        if (!Set.of("QUEUED", "WAITING_MANIFEST", "RUNNING", "HOLD", "COMPLETED", "FAILED", "REJECTED").contains(state)) {
            throw new IllegalArgumentException("unknown task state");
        }
        if (!payload.path("task_revision").isIntegralNumber() || payload.path("task_revision").asLong() < 0) {
            throw new IllegalArgumentException("invalid revision");
        }
        long revision = payload.path("task_revision").asLong();
        boolean owns = db.sql("SELECT 1 FROM pickups p JOIN orders o USING(order_id) WHERE pickup_id=? AND p.order_id=? AND site_id=?")
                .param(pickup).param(order).param(site).query().optionalValue().isPresent();
        if (!owns) throw new IllegalArgumentException("unknown reservation");
        var prior = db.sql("SELECT * FROM control_task_views WHERE task_ref=?").param(task).query().listOfRows().stream().findFirst();
        if (prior.isPresent() && !pickup.equals(prior.get().get("pickup_id"))) throw new IllegalArgumentException("task reservation mismatch");
        boolean terminal = prior.isPresent() && Set.of("COMPLETED", "FAILED", "REJECTED").contains(prior.get().get("state"));
        if (!terminal) {
            db.sql("""
                    INSERT INTO control_task_views(task_ref,pickup_id,site_id,task_revision,state,progress_payload,observed_at,synced_at)
                    VALUES (?,?,?,?,?,?::jsonb,?,now()) ON CONFLICT(task_ref) DO UPDATE
                    SET task_revision=excluded.task_revision,state=excluded.state,progress_payload=excluded.progress_payload,
                        observed_at=excluded.observed_at,synced_at=now(),updated_at=now()
                    WHERE control_task_views.task_revision < excluded.task_revision""")
                    .param(task).param(pickup).param(site).param(revision).param(state).param(payload.toString())
                    .param(OffsetDateTime.parse(text(message, "sent_at"))).update();
        }
        db.sql("INSERT INTO control_link_inbox(message_id,site_id,message) VALUES (?,?,?::jsonb)")
                .param(id).param(site).param(raw).update();
        return envelope(site, "message.ack", Map.of("message_id", id));
    }
}
