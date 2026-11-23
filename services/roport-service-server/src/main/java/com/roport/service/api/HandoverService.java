package com.roport.service.api;

import java.time.Clock;
import java.time.Duration;
import java.time.OffsetDateTime;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpStatus;
import org.springframework.jdbc.core.simple.JdbcClient;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import tools.jackson.databind.ObjectMapper;

/** Explicit receipt confirmation; a QR lookup or a robot arrival never calls this. */
@Service
class HandoverService {
    private final JdbcClient db;
    private final ObjectMapper json;
    private final Clock clock;
    private final Duration verificationAge;

    HandoverService(JdbcClient db, ObjectMapper json, Clock clock,
            @Value("${service.handover.verification-age:PT5M}") Duration verificationAge) {
        if (verificationAge.isNegative() || verificationAge.isZero()) throw new IllegalArgumentException("verification age");
        this.db = db; this.json = json; this.clock = clock; this.verificationAge = verificationAge;
    }

    private ApiException conflict(String code) {
        return new ApiException(HttpStatus.CONFLICT, code, "예약·카트 배정·도착 상태를 다시 확인해 주세요.");
    }

    @Transactional
    public Map<String, Object> confirm(String cart, String verificationId, String requestId, Integer followCard) {
        // Serializes retry IDs and receipt assignment; no device or external network calls in this transaction.
        db.sql("SELECT pg_advisory_xact_lock(71519006)").query().singleValue();
        var previous = db.sql("SELECT r.receipt_id,r.follow_card,a.verification_id,a.cart_ref FROM receipts r JOIN handover_approvals a USING(approval_id) WHERE confirmation_request_id=?")
                .param(requestId).query().listOfRows().stream().findFirst();
        if (previous.isPresent()) {
            var row = previous.get();
            if (!java.util.Objects.equals(followCard, row.get("follow_card")) || !cart.equals(row.get("cart_ref")) || !verificationId.equals(row.get("verification_id"))) throw conflict("IDEMPOTENCY_CONFLICT");
            return result((String) row.get("receipt_id"));
        }
        var found = db.sql("""
                SELECT v.*,p.order_id,o.revision,o.business_status,o.site_id FROM verifications v
                JOIN pickups p USING(pickup_id) JOIN orders o USING(order_id)
                WHERE v.verification_id=? FOR UPDATE OF p,o
                """).param(verificationId).query().listOfRows().stream().findFirst();
        if (found.isEmpty()) throw conflict("VERIFICATION_REQUIRED");
        var verified = found.get();
        var at = (java.sql.Timestamp) verified.get("verified_at");
        if (!"VERIFIED".equals(verified.get("result")) || !("cart:" + cart).equals(verified.get("actor_ref"))
                || at == null || at.toInstant().isAfter(clock.instant())
                || at.toInstant().plus(verificationAge).isBefore(clock.instant())) throw conflict("VERIFICATION_REQUIRED");
        if (verified.get("credential_id") != null && db.sql("SELECT 1 FROM qr_credentials WHERE credential_id=? AND revoked_at IS NULL AND (expires_at IS NULL OR expires_at>?)")
                .param(verified.get("credential_id")).param(OffsetDateTime.ofInstant(clock.instant(), java.time.ZoneOffset.UTC))
                .query().optionalValue().isEmpty()) throw conflict("VERIFICATION_REQUIRED");
        if (!"CONFIRMED".equals(verified.get("business_status"))) throw conflict("ORDER_NOT_ACTIVE");
        String pickup = (String) verified.get("pickup_id"), order = (String) verified.get("order_id");
        var views = db.sql("SELECT * FROM control_task_views WHERE pickup_id=? AND site_id=? AND state='RUNNING' FOR UPDATE")
                .param(pickup).param(verified.get("site_id")).query().listOfRows();
        if (views.size() != 1) throw conflict("CART_NOT_ARRIVED");
        var view = views.getFirst();
        var progress = json.readTree(view.get("progress_payload").toString()).path("progress").path("packages");
        List<String> packages = new java.util.ArrayList<>();
        for (var p : progress) {
            if (cart.equals(p.path("robot").asString()) && "await_receipt".equals(p.path("stage").asString())) {
                packages.add(p.path("package_id").asString());
            }
        }
        if (packages.isEmpty()) throw conflict("CART_NOT_ARRIVED");
        for (String id : packages) {
            if (db.sql("SELECT 1 FROM packages WHERE package_id=? AND order_id=? AND inactive_at IS NULL AND manifest_locked_at IS NOT NULL AND NOT EXISTS (SELECT 1 FROM receipt_packages WHERE package_id=?)")
                    .param(id).param(order).param(id).query().optionalValue().isEmpty()) throw conflict("PACKAGE_NOT_AVAILABLE");
        }
        String approval = UUID.randomUUID().toString(), receipt = UUID.randomUUID().toString();
        db.sql("UPDATE handover_approvals SET revoked_at=now() WHERE pickup_id=? AND consumed_at IS NULL AND revoked_at IS NULL")
                .param(pickup).update();
        // This approval is consumed by the same explicit confirmation, never by the verification API.
        db.sql("INSERT INTO handover_approvals(approval_id,pickup_id,order_id,verification_id,task_ref,cart_ref,order_revision,manifest_hash,consumed_at) VALUES(?,?,?,?,?,?,?,?,now())")
                .param(approval).param(pickup).param(order).param(verificationId).param(view.get("task_ref")).param(cart)
                .param(verified.get("revision")).param(QrTokens.hash(json.writeValueAsString(packages.stream().sorted().toList()))).update();
        db.sql("INSERT INTO receipts(receipt_id,order_id,pickup_id,approval_id,confirmation_request_id,confirmed_by,channel,received_at,order_revision,follow_card) VALUES(?,?,?,?,?,?,'CART',now(),?,?)")
                .param(receipt).param(order).param(pickup).param(approval).param(requestId).param("cart:" + cart).param(verified.get("revision")).param(followCard).update();
        for (String id : packages) {
            db.sql("INSERT INTO receipt_packages(receipt_id,order_id,package_id,package_revision) SELECT ?,order_id,package_id,package_revision FROM packages WHERE package_id=?")
                    .param(receipt).param(id).update();
        }
        boolean complete = db.sql("SELECT NOT EXISTS (SELECT 1 FROM packages p WHERE order_id=? AND inactive_at IS NULL AND NOT EXISTS (SELECT 1 FROM receipt_packages r WHERE r.package_id=p.package_id))")
                .param(order).query(Boolean.class).single();
        if (complete) {
            db.sql("UPDATE orders SET business_status='RECEIVED' WHERE order_id=?").param(order).update();
            db.sql("UPDATE pickups SET status='CLOSED' WHERE pickup_id=?").param(pickup).update();
        }
        return result(receipt);
    }

    private Map<String, Object> result(String receipt) {
        var row = db.sql("SELECT r.receipt_id,o.business_status FROM receipts r JOIN orders o USING(order_id) WHERE receipt_id=?")
                .param(receipt).query().singleRow();
        return Map.of("receiptId", receipt, "orderComplete", "RECEIVED".equals(row.get("business_status")),
                "packageIds", db.sql("SELECT package_id FROM receipt_packages WHERE receipt_id=? ORDER BY package_id").param(receipt).query(String.class).list());
    }
}
