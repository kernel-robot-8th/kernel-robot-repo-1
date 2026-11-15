package com.roport.service.api;

import java.time.Clock;
import java.time.Instant;
import java.time.OffsetDateTime;
import java.time.ZoneOffset;
import java.util.Set;
import java.util.UUID;
import org.springframework.http.HttpStatus;
import org.springframework.jdbc.core.simple.JdbcClient;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

/**
 * 고객 예약 QR 발급(SR-18). 발급할 때마다 새 토큰을 만들고 같은 예약의 이전 토큰은 폐기한다.
 * 만료 시각(expires_at)은 팀이 정할 값이라 비워 둔다. QR 발급은 인도 승인이 아니다.
 */
@Service
class QrService {
    /** QR을 발급할 수 있는 예약 상태(주문은 CONFIRMED). 취소 대기(CANCEL_PENDING)는 제외한다. */
    static final Set<String> QR_ALLOWED = Set.of("REQUESTED", "CONFIRMED", "CHANGE_PENDING");
    static final String QR_ALLOWED_SQL = "('REQUESTED', 'CONFIRMED', 'CHANGE_PENDING')";

    record QrToken(String pickupId, String token, Instant issuedAt) {}

    private final JdbcClient db;
    private final Clock clock;

    QrService(JdbcClient db, Clock clock) {
        this.db = db;
        this.clock = clock;
    }

    /** 잠금 순서는 orders → pickups(서비스 DB 공통 규칙). 다른 회원의 예약은 존재 여부를 드러내지 않고 404. */
    @Transactional
    QrToken issue(String memberId, String pickupId) {
        String orderId = db.sql("SELECT p.order_id FROM pickups p JOIN orders o USING (order_id)"
                        + " WHERE p.pickup_id = ? AND o.member_id = ?")
                .param(pickupId).param(memberId)
                .query(String.class).optional()
                .orElseThrow(() -> ApiException.notFound("PICKUP_NOT_FOUND"));
        String businessStatus = db.sql("SELECT business_status FROM orders WHERE order_id = ? FOR UPDATE")
                .param(orderId).query(String.class).single();
        boolean allowed = "CONFIRMED".equals(businessStatus)
                && db.sql("SELECT 1 FROM pickups WHERE pickup_id = ? AND status IN " + QR_ALLOWED_SQL + " FOR UPDATE")
                        .param(pickupId).query().optionalValue().isPresent();
        if (!allowed) {
            throw new ApiException(HttpStatus.CONFLICT, "PICKUP_NOT_ACTIVE", "QR을 발급할 수 없는 예약입니다.");
        }
        OffsetDateTime now = OffsetDateTime.ofInstant(clock.instant(), ZoneOffset.UTC);
        db.sql("UPDATE qr_credentials SET revoked_at = ?, updated_at = ? WHERE pickup_id = ? AND revoked_at IS NULL")
                .param(now).param(now).param(pickupId).update();
        String token = QrTokens.newToken();
        db.sql("INSERT INTO qr_credentials (credential_id, token_hash, pickup_id, issued_at) VALUES (?, ?, ?, ?)")
                .param(UUID.randomUUID().toString()).param(QrTokens.hash(token)).param(pickupId).param(now)
                .update();
        return new QrToken(pickupId, token, now.toInstant());
    }
}
