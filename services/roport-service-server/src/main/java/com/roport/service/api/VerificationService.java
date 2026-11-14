package com.roport.service.api;

import com.roport.service.api.PickupService.Item;
import com.roport.service.api.PickupService.Location;
import java.time.Clock;
import java.time.Instant;
import java.time.OffsetDateTime;
import java.time.ZoneOffset;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.UUID;
import org.springframework.http.HttpStatus;
import org.springframework.jdbc.core.simple.JdbcClient;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

/**
 * 카트 패드의 예약 확인(SR-18-2). QR 토큰 또는 예약번호로 예약을 찾아 verifications에 결과를 기록한다.
 *
 * <p>확인(VERIFIED)은 인도 승인이 아니다. 이 카트에 배정된 예약인지는 관제의 배정 정보가 있어야 판단할 수 있어
 * 아직 대조하지 않으며(cartAssignmentChecked=false), handover_approvals·receipts는 만들지 않는다.
 *
 * <p>같은 verificationRequestId 재전송은 저장된 결과를 돌려준다. 같은 ID로 다른 카트·방법·예약·QR 토큰을 보내면 409.
 * 예약을 찾지 못하면 verifications.pickup_id가 NOT NULL이라 기록하지 않고 404를 돌려준다.
 */
@Service
class VerificationService {
    enum Method { QR, RESERVATION_NO }

    record VerifyRequest(String verificationRequestId, Method method, String value) {}

    record FlightNow(String flightNo, Instant boardingStart, Instant boardingEnd, String gateCode) {}

    record VerifiedPickup(String pickupId, String reservationNo, String orderNo, String customerName,
                          Location location, FlightNow flight, List<Item> items, int packageCount) {}

    record Verification(String verificationId, String verificationRequestId, String result, String reasonCode,
                        Instant verifiedAt, boolean cartAssignmentChecked, VerifiedPickup pickup) {}

    record Outcome(Verification verification, boolean created) {}

    private record Target(String pickupId, String credentialId, String reasonCode) {}

    private record Stored(String verificationId, String verificationRequestId, String pickupId, String credentialId,
                          String method, String actorRef, String result,
                          String reasonCode, Instant verifiedAt) {}

    private final JdbcClient db;
    private final Clock clock;

    VerificationService(JdbcClient db, Clock clock) {
        this.db = db;
        this.clock = clock;
    }

    @Transactional
    Outcome verify(String cartId, VerifyRequest req) {
        String actorRef = "cart:" + cartId;
        Instant now = clock.instant();
        Target target = resolve(req, now);
        Optional<Stored> existing = stored(req.verificationRequestId());
        if (existing.isPresent()) {
            return replay(existing.get(), actorRef, req.method(), target);
        }
        boolean verified = target.reasonCode() == null;
        OffsetDateTime at = OffsetDateTime.ofInstant(now, ZoneOffset.UTC);
        int rows = db.sql("""
                INSERT INTO verifications (verification_id, verification_request_id, pickup_id, method, result,
                    verified_at, actor_ref, credential_id, reason_code)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT (verification_request_id) DO NOTHING""")
                .param(UUID.randomUUID().toString()).param(req.verificationRequestId()).param(target.pickupId())
                .param(req.method().name()).param(verified ? "VERIFIED" : "REJECTED")
                .param(verified ? at : null).param(actorRef).param(target.credentialId()).param(target.reasonCode())
                .update();
        Stored row = stored(req.verificationRequestId()).orElseThrow();
        if (rows == 0) {
            // 같은 ID의 동시 요청이 먼저 저장했다. 그 결과를 같은 규칙으로 돌려준다.
            return replay(row, actorRef, req.method(), target);
        }
        return new Outcome(toResponse(row), true);
    }

    private Outcome replay(Stored row, String actorRef, Method method, Target target) {
        if (!row.actorRef().equals(actorRef) || !row.method().equals(method.name())
                || !row.pickupId().equals(target.pickupId())
                || !Objects.equals(row.credentialId(), target.credentialId())) {
            throw new ApiException(HttpStatus.CONFLICT, "IDEMPOTENCY_CONFLICT",
                    "같은 요청 ID로 다른 확인 요청을 보낼 수 없습니다.");
        }
        return new Outcome(toResponse(row), false);
    }

    /** 예약을 찾고 거절 사유를 정한다. 찾지 못하면 404(기록 없음). */
    private Target resolve(VerifyRequest req, Instant now) {
        String pickupId;
        String credentialId = null;
        String reason = null;
        if (req.method() == Method.QR) {
            record Cred(String credentialId, String pickupId, OffsetDateTime revokedAt, OffsetDateTime expiresAt) {}
            Cred cred = db.sql("SELECT credential_id, pickup_id, revoked_at, expires_at FROM qr_credentials WHERE token_hash = ?")
                    .param(QrTokens.hash(req.value()))
                    .query((rs, n) -> new Cred(rs.getString(1), rs.getString(2),
                            rs.getObject(3, OffsetDateTime.class), rs.getObject(4, OffsetDateTime.class)))
                    .optional()
                    .filter(c -> c.pickupId() != null) // 회원·주문 범위 QR은 이 API에서 다루지 않는다
                    .orElseThrow(VerificationService::notFound);
            pickupId = cred.pickupId();
            credentialId = cred.credentialId();
            if (cred.revokedAt() != null) {
                reason = "QR_REVOKED";
            } else if (cred.expiresAt() != null && !cred.expiresAt().toInstant().isAfter(now)) {
                reason = "QR_EXPIRED";
            }
        } else {
            pickupId = db.sql("SELECT pickup_id FROM pickups WHERE reservation_no = ?").param(req.value())
                    .query(String.class).optional()
                    .orElseThrow(VerificationService::notFound);
        }
        if (reason == null) {
            record State(String status, String businessStatus) {}
            State s = db.sql("SELECT p.status, o.business_status FROM pickups p JOIN orders o USING (order_id) WHERE p.pickup_id = ?")
                    .param(pickupId).query((rs, n) -> new State(rs.getString(1), rs.getString(2))).single();
            if ("RECEIVED".equals(s.businessStatus())) {
                reason = "ALREADY_RECEIVED";
            } else if (!QrService.QR_ALLOWED.contains(s.status()) || !"CONFIRMED".equals(s.businessStatus())) {
                reason = "PICKUP_NOT_ACTIVE";
            }
        }
        return new Target(pickupId, credentialId, reason);
    }

    private static ApiException notFound() {
        return ApiException.notFound("RESERVATION_NOT_FOUND");
    }

    private Optional<Stored> stored(String verificationRequestId) {
        return db.sql("""
                SELECT verification_id, verification_request_id, pickup_id, credential_id, method, actor_ref, result, reason_code, verified_at
                FROM verifications WHERE verification_request_id = ?""")
                .param(verificationRequestId)
                .query((rs, n) -> {
                    OffsetDateTime at = rs.getObject("verified_at", OffsetDateTime.class);
                    return new Stored(rs.getString("verification_id"), rs.getString("verification_request_id"),
                            rs.getString("pickup_id"), rs.getString("credential_id"),
                            rs.getString("method"), rs.getString("actor_ref"), rs.getString("result"),
                            rs.getString("reason_code"), at == null ? null : at.toInstant());
                })
                .optional();
    }

    private Verification toResponse(Stored row) {
        VerifiedPickup pickup = "VERIFIED".equals(row.result()) ? pickup(row.pickupId()) : null;
        return new Verification(row.verificationId(), row.verificationRequestId(), row.result(), row.reasonCode(), row.verifiedAt(), false, pickup);
    }

    /** 확인된 예약의 표시 정보. 항공편은 예약 당시 사본이 아니라 최신 값(게이트 변경 반영)을 쓴다. */
    private VerifiedPickup pickup(String pickupId) {
        record Row(String reservationNo, String orderId, String orderNo, String customerName, Location location,
                   FlightNow flight) {}
        Row r = db.sql("""
                SELECT p.reservation_no, o.order_id, o.order_no, m.display_name, l.location_id, l.name,
                    f.flight_no, f.boarding_start, f.boarding_end, f.gate_code
                FROM pickups p JOIN orders o USING (order_id) JOIN members m USING (member_id)
                    JOIN pickup_locations l USING (location_id) JOIN flights f USING (flight_id)
                WHERE p.pickup_id = ?""")
                .param(pickupId)
                .query((rs, n) -> new Row(rs.getString("reservation_no"), rs.getString("order_id"),
                        rs.getString("order_no"), rs.getString("display_name"),
                        new Location(rs.getString("location_id"), rs.getString("name")),
                        new FlightNow(rs.getString("flight_no"), PickupService.ts(rs, "boarding_start"),
                                PickupService.ts(rs, "boarding_end"), rs.getString("gate_code"))))
                .single();
        List<Item> items = db.sql("SELECT item_id, product_name, quantity FROM order_items WHERE order_id = ? ORDER BY item_id")
                .param(r.orderId())
                .query((rs, n) -> new Item(rs.getString(1), rs.getString(2), rs.getInt(3))).list();
        // 포장 구성이 아직 없으면 0이다(추정하지 않는다). 비활성(재포장·폐기) 포장은 세지 않는다.
        int packages = db.sql("SELECT count(*) FROM packages WHERE order_id = ? AND inactive_at IS NULL")
                .param(r.orderId()).query(Integer.class).single();
        return new VerifiedPickup(pickupId, r.reservationNo(), r.orderNo(), r.customerName(), r.location(),
                r.flight(), items, packages);
    }
}
