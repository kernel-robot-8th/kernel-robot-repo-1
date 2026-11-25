package com.roport.service.control;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.WebSocket;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CompletionStage;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.web.server.LocalServerPort;
import org.springframework.jdbc.core.simple.JdbcClient;
import org.springframework.transaction.PlatformTransactionManager;
import org.springframework.transaction.support.TransactionTemplate;
import tools.jackson.databind.ObjectMapper;

@SpringBootTest(webEnvironment = SpringBootTest.WebEnvironment.RANDOM_PORT, properties = {
        "service.control.enabled=true", "service.control.site=LINK-TEST", "service.control.token=isolated-test-token"})
class ControlExchangeTest {
    @Autowired ControlExchange exchange;
    @Autowired JdbcClient db;
    @Autowired ObjectMapper json;
    @Autowired PlatformTransactionManager transactions;
    @LocalServerPort int port;
    String id;

    @BeforeEach
    void seed() {
        id = UUID.randomUUID().toString();
        db.sql("INSERT INTO members(member_id) VALUES(?)").param(id).update();
        db.sql("INSERT INTO orders(order_id,order_no,member_id,site_id,revision,business_status,ordered_at) VALUES(?,?,?,'LINK-TEST',0,'CONFIRMED',now())")
                .param(id).param(id).param(id).update();
        db.sql("INSERT INTO pickup_locations(location_id,site_id,name,control_point_ref,is_active) VALUES(?,'LINK-TEST','test',?,true)")
                .param(id).param(id).update();
        db.sql("INSERT INTO flights(flight_id,flight_no,service_date,boarding_start,boarding_end,departure_at,gate_code,revision) VALUES(?,?,CURRENT_DATE,now(),now(),now(),'G1',0)")
                .param(id).param(id).update();
        db.sql("INSERT INTO pickups(pickup_id,reservation_no,order_id,location_id,flight_id,flight_revision,flight_snapshot,pickup_at,channel,status,revision) VALUES(?,?,?,?,?,0,'{}',now(),'WEB','REQUESTED',0)")
                .param(id).param(id).param(id).param(id).param(id).update();
        db.sql("INSERT INTO packages(package_id,order_id,package_revision,label_ref,manifest_locked_at) VALUES(?,?,0,'label',now())")
                .param(id).param(id).update();
    }

    Map<String,Object> event(String kind, String state, long revision) {
        var result = exchange.envelope("LINK-TEST", kind, Map.of("reservation_id", id, "order_id", id,
                "task_revision", revision, "state", state, "progress", Map.of("node", "N12")));
        result.put("task_id", id);
        result.put("source", "control-01");
        result.put("target", "service-01");
        return result;
    }

    @Test
    void sharedCandidateExamplesAndInvalidDirection() throws Exception {
        var examples = json.readTree(java.nio.file.Files.readString(java.nio.file.Path.of("../../interfaces/wss/candidate/examples.json")));
        for (var row : examples) {
            var message = row.path("message");
            if (message.path("source").asString().equals("control-01")) ControlExchange.validate(message);
        }
        var valid = event("task.accepted", "QUEUED", 0);
        for (String key : java.util.List.of("source", "target", "schema_version", "sent_at")) {
            var bad = new java.util.LinkedHashMap<>(valid);
            bad.put(key, "invalid");
            assertThatThrownBy(() -> exchange.receive("LINK-TEST", json.writeValueAsString(bad))).isInstanceOf(RuntimeException.class);
        }
        assertThat(db.sql("SELECT count(*) FROM control_link_inbox WHERE message_id=?").param(valid.get("message_id")).query(Long.class).single()).isZero();
    }

    @Test
    void reservationOutboxSharesTransactionAndUsesLockedPackages() {
        new TransactionTemplate(transactions).executeWithoutResult(tx -> {
            exchange.reservationCreated(id);
            tx.setRollbackOnly();
        });
        assertThat(db.sql("SELECT count(*) FROM control_link_outbox WHERE source_ref=?").param("reservation:"+id).query(Long.class).single()).isZero();
        exchange.reservationCreated(id);
        exchange.reservationCreated(id);
        String raw = db.sql("SELECT message::text FROM control_link_outbox WHERE source_ref=?").param("reservation:"+id).query(String.class).single();
        var message = json.readTree(raw);
        assertThat(message.path("payload").path("packages").get(0).path("package_id").asString()).isEqualTo(id);
        assertThat(message.path("payload").path("pickup_point_id").asString()).isEqualTo(id);
        var ack = exchange.envelope("LINK-TEST", "message.ack", Map.of("message_id", message.path("message_id").asString()));
        ack.put("source", "control-01");
        ack.put("target", "service-01");
        exchange.receive("LINK-TEST", json.writeValueAsString(ack));
        assertThat(db.sql("SELECT acknowledged_at IS NOT NULL FROM control_link_outbox WHERE source_ref=?").param("reservation:"+id).query(Boolean.class).single()).isTrue();
    }

    @Test
    void revisionedEventsDeduplicateAndDoNotRecordCustomerReceipt() {
        var accepted = event("task.accepted", "QUEUED", 0);
        String raw = json.writeValueAsString(accepted);
        exchange.receive("LINK-TEST", raw);
        exchange.receive("LINK-TEST", raw);
        accepted.put("payload", Map.of("reservation_id", id, "order_id", id, "task_revision", 99, "state", "QUEUED", "progress", Map.of()));
        assertThatThrownBy(() -> exchange.receive("LINK-TEST", json.writeValueAsString(accepted)))
                .isInstanceOf(IllegalArgumentException.class).hasMessageContaining("reused");
        exchange.receive("LINK-TEST", json.writeValueAsString(event("task.completed", "COMPLETED", 2)));
        exchange.receive("LINK-TEST", json.writeValueAsString(event("task.progress", "RUNNING", 1)));
        exchange.receive("LINK-TEST", json.writeValueAsString(event("task.progress", "RUNNING", 3)));
        assertThat(db.sql("SELECT state FROM control_task_views WHERE task_ref=?").param(id).query(String.class).single()).isEqualTo("COMPLETED");
        assertThat(db.sql("SELECT business_status FROM orders WHERE order_id=?").param(id).query(String.class).single()).isEqualTo("CONFIRMED");
        assertThat(db.sql("SELECT count(*) FROM receipts WHERE order_id=?").param(id).query(Long.class).single()).isZero();
    }

    @Test
    void markerReuseRequiresEmptyBoxReturnAndPreservesPackageHistory() {
        String next = id + "-next";
        db.sql("INSERT INTO packages(package_id,order_id,package_revision,label_ref) VALUES(?,?,0,'test')")
                .param(next).param(id).update();
        String assign = "INSERT INTO package_markers(package_id,marker) VALUES(?, '16')";
        db.sql(assign).param(id).update();
        assertThatThrownBy(() -> db.sql(assign).param(next).update())
                .isInstanceOf(org.springframework.dao.DuplicateKeyException.class);
        exchange.reservationCreated(id);
        var message = json.readTree(db.sql("SELECT message::text FROM control_link_outbox WHERE source_ref=?")
                .param("reservation:" + id).query(String.class).single());
        var pack = message.path("payload").path("packages").get(0);
        assertThat(pack.path("package_id").asString()).isEqualTo(id);
        assertThat(pack.path("marker").asString()).isEqualTo("16");
        db.sql("UPDATE package_markers SET empty_box_returned_at=now() WHERE package_id=?").param(id).update();
        db.sql(assign).param(next).update();
        assertThat(db.sql("SELECT count(*) FROM package_markers WHERE marker='16' AND package_id IN (?,?)")
                .param(id).param(next).query(Integer.class).single()).isEqualTo(2);
        db.sql("UPDATE package_markers SET empty_box_returned_at=now() WHERE package_id=?").param(next).update();
    }

    @Test
    void publishesOnlyCommittedPackageReceiptEvidence() {
        exchange.receive("LINK-TEST", json.writeValueAsString(event("task.progress", "RUNNING", 1)));
        db.sql("INSERT INTO verifications(verification_id,verification_request_id,pickup_id,method,result,actor_ref) VALUES(?,?,?,'test','VERIFIED','test')")
                .param(id).param(id).param(id).update();
        db.sql("INSERT INTO handover_approvals(approval_id,pickup_id,order_id,verification_id,task_ref,cart_ref,order_revision,manifest_hash) VALUES(?,?,?,?,?,'cart-test',0,'test')")
                .param(id).param(id).param(id).param(id).param(id).update();
        db.sql("INSERT INTO receipts(receipt_id,order_id,pickup_id,approval_id,confirmation_request_id,confirmed_by,channel,received_at,order_revision) VALUES(?,?,?,?,?,?,'test',now(),0)")
                .param(id).param(id).param(id).param(id).param(id).param(id).update();
        exchange.publishReceipts("LINK-TEST");
        assertThat(db.sql("SELECT count(*) FROM control_link_outbox WHERE source_ref=?").param("receipt:"+id).query(Long.class).single()).isZero();
        db.sql("INSERT INTO receipt_packages(receipt_id,order_id,package_id,package_revision) VALUES(?,?,?,0)")
                .param(id).param(id).param(id).update();
        exchange.publishReceipts("LINK-TEST");
        exchange.publishReceipts("LINK-TEST");
        var message = json.readTree(db.sql("SELECT message::text FROM control_link_outbox WHERE source_ref=?")
                .param("receipt:"+id).query(String.class).single());
        assertThat(message.path("payload").path("package_ids").get(0).asString()).isEqualTo(id);
        assertThat(message.path("task_id").asString()).isEqualTo(id);
        assertThat(message.path("payload").has("follow_mode")).isFalse();
        assertThat(message.path("payload").has("follow_card")).isFalse();
    }

    @Test
    void explicitAppearanceReceiptPublishesModeWithoutMarkerCard() {
        exchange.receive("LINK-TEST", json.writeValueAsString(event("task.progress", "RUNNING", 1)));
        db.sql("INSERT INTO verifications(verification_id,verification_request_id,pickup_id,method,result,actor_ref) VALUES(?,?,?,'test','VERIFIED','test')")
                .param(id).param(id).param(id).update();
        db.sql("INSERT INTO handover_approvals(approval_id,pickup_id,order_id,verification_id,task_ref,cart_ref,order_revision,manifest_hash) VALUES(?,?,?,?,?,'cart-test',0,'test')")
                .param(id).param(id).param(id).param(id).param(id).update();
        db.sql("INSERT INTO receipts(receipt_id,order_id,pickup_id,approval_id,confirmation_request_id,confirmed_by,channel,received_at,order_revision,follow_mode) VALUES(?,?,?,?,?,?,'test',now(),0,'appearance')")
                .param(id).param(id).param(id).param(id).param(id).param(id).update();
        db.sql("INSERT INTO receipt_packages(receipt_id,order_id,package_id,package_revision) VALUES(?,?,?,0)")
                .param(id).param(id).param(id).update();
        exchange.publishReceipts("LINK-TEST");
        var message = json.readTree(db.sql("SELECT message::text FROM control_link_outbox WHERE source_ref=?")
                .param("receipt:" + id).query(String.class).single());
        assertThat(message.path("payload").path("follow_mode").asString()).isEqualTo("appearance");
        assertThat(message.path("payload").has("follow_card")).isFalse();
    }

    @Test
    void replacementClosesOldOwnerWith4001AndOldCloseCannotRemoveNewOwner() throws Exception {
        URI uri = URI.create("ws://127.0.0.1:" + port + "/ws/v1/sites/LINK-TEST/control");
        HttpClient client = HttpClient.newHttpClient();
        BlockingQueue<Integer> closed = new LinkedBlockingQueue<>();
        WebSocket first = client.newWebSocketBuilder().header("Authorization", "Bearer isolated-test-token")
                .buildAsync(uri, new WebSocket.Listener() {
                    @Override public CompletionStage<?> onClose(WebSocket ws, int status, String reason) {
                        closed.add(status); return null;
                    }
                }).join();
        BlockingQueue<String> responses = new LinkedBlockingQueue<>();
        WebSocket second = client.newWebSocketBuilder().header("Authorization", "Bearer isolated-test-token")
                .buildAsync(uri, new WebSocket.Listener() {
                    StringBuilder parts = new StringBuilder();
                    @Override public CompletionStage<?> onText(WebSocket ws, CharSequence text, boolean last) {
                        parts.append(text);
                        if (last) { responses.add(parts.toString()); parts.setLength(0); }
                        ws.request(1); return null;
                    }
                }).join();
        try {
            assertThat(closed.poll(5, TimeUnit.SECONDS)).isEqualTo(4001);
            first.abort();
            var event = event("task.accepted", "QUEUED", 0);
            second.sendText(json.writeValueAsString(event), true).join();
            boolean acked = false;
            long until = System.nanoTime() + TimeUnit.SECONDS.toNanos(5);
            while (!acked && System.nanoTime() < until) {
                String raw = responses.poll(1, TimeUnit.SECONDS);
                acked = raw != null && json.readTree(raw).path("payload").path("message_id").asString().equals(event.get("message_id"));
            }
            assertThat(acked).isTrue();
        } finally { first.abort(); second.sendClose(WebSocket.NORMAL_CLOSURE, "done").join(); }
    }

    @Test
    void websocketAuthenticatesAndAcknowledgesAfterViewCommit() throws Exception {
        URI uri = URI.create("ws://127.0.0.1:"+port+"/ws/v1/sites/LINK-TEST/control");
        HttpClient client = HttpClient.newHttpClient();
        assertThatThrownBy(() -> client.newWebSocketBuilder().buildAsync(uri, new WebSocket.Listener() {}).join())
                .hasCauseInstanceOf(java.net.http.WebSocketHandshakeException.class);
        BlockingQueue<String> incoming = new LinkedBlockingQueue<>();
        WebSocket socket = client.newWebSocketBuilder().header("Authorization", "Bearer isolated-test-token")
                .buildAsync(uri, new WebSocket.Listener() {
                    StringBuilder parts = new StringBuilder();
                    @Override public void onOpen(WebSocket ws) { ws.request(1); }
                    @Override public CompletionStage<?> onText(WebSocket ws, CharSequence text, boolean last) {
                        parts.append(text);
                        if (last) { incoming.add(parts.toString()); parts.setLength(0); }
                        ws.request(1);
                        return null;
                    }
                }).join();
        try {
            var event = event("task.accepted", "QUEUED", 0);
            socket.sendText(json.writeValueAsString(event), true).join();
            String ack = null;
            long until = System.nanoTime() + TimeUnit.SECONDS.toNanos(5);
            while (System.nanoTime() < until) {
                var next = incoming.poll(1, TimeUnit.SECONDS);
                if (next != null && "message.ack".equals(json.readTree(next).path("type").asString())) { ack = next; break; }
            }
            assertThat(ack).isNotNull();
            assertThat(json.readTree(ack).path("payload").path("message_id").asString()).isEqualTo(event.get("message_id"));
            assertThat(db.sql("SELECT count(*) FROM control_task_views WHERE task_ref=?").param(id).query(Long.class).single()).isEqualTo(1);
        } finally {
            socket.sendClose(WebSocket.NORMAL_CLOSURE, "done").join();
        }
    }
}
