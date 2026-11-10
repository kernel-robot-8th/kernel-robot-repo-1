package com.roport.service.api;

import java.security.SecureRandom;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.time.Clock;
import java.time.Duration;
import java.time.Instant;
import java.time.LocalDate;
import java.time.OffsetDateTime;
import java.time.ZoneOffset;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.UUID;
import org.springframework.http.HttpStatus;
import org.springframework.jdbc.core.simple.JdbcClient;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import tools.jackson.core.type.TypeReference;
import tools.jackson.databind.ObjectMapper;

/** 조회와 예약 생성. 예약은 status='REQUESTED'(현장 배정 전)만 만든다. 카트 확보·관제 수락·수령 완료가 아니다. */
@Service
class PickupService {
    // D5 예약 가능 시간 규칙
    static final Duration MIN_LEAD = Duration.ofMinutes(60);
    static final Duration SLOT = Duration.ofMinutes(30);
    // pickups_one_active_per_order 인덱스와 같은 집합
    static final String ACTIVE_SQL = "('REQUESTED', 'CONFIRMED', 'CHANGE_PENDING', 'CANCEL_PENDING')";
    static final String RESERVATION_ALPHABET = "23456789ABCDEFGHJKLMNPQRSTUVWXYZ"; // 0,O,1,I 제외 (D9)

    record Item(String itemId, String productName, int quantity) {}

    record ActivePickup(String pickupId, String reservationNo, String status) {}

    record Order(String orderId, String orderNo, Instant orderedAt, String businessStatus, List<Item> items,
                 ActivePickup activePickup) {}

    record Location(String locationId, String name) {}

    record Flight(String flightId, String flightNo, LocalDate serviceDate, Instant boardingStart,
                  Instant boardingEnd, Instant departureAt, String gateCode) {}

    record Pickup(String pickupId, String reservationNo, String orderId, String orderNo, Location location,
                  Map<String, Object> flight, Instant pickupAt, String status, long revision, Instant createdAt) {}

    record CreatePickup(String requestId, String orderId, String locationId, String flightId, Instant pickupAt) {}

    record Created(Pickup pickup, boolean created) {}

    private record OrderRow(String siteId, String businessStatus) {}

    private final JdbcClient db;
    private final Clock clock;
    private final ObjectMapper json;
    private final SecureRandom random = new SecureRandom();

    PickupService(JdbcClient db, Clock clock, ObjectMapper json) {
        this.db = db;
        this.clock = clock;
        this.json = json;
    }

    static Instant ts(ResultSet rs, String col) throws SQLException {
        return rs.getObject(col, OffsetDateTime.class).toInstant();
    }

    List<Order> orders(String memberId) {
        String mine = " JOIN orders o USING (order_id) WHERE o.member_id = ? AND o.business_status = 'CONFIRMED'";
        Map<String, List<Item>> items = new LinkedHashMap<>();
        db.sql("SELECT order_id, item_id, product_name, quantity FROM order_items" + mine + " ORDER BY item_id")
                .param(memberId)
                .query(rs -> {
                    items.computeIfAbsent(rs.getString("order_id"), k -> new ArrayList<>()).add(new Item(
                            rs.getString("item_id"), rs.getString("product_name"), rs.getInt("quantity")));
                });
        Map<String, ActivePickup> active = new LinkedHashMap<>();
        db.sql("SELECT order_id, pickup_id, reservation_no, status FROM pickups" + mine + " AND status IN " + ACTIVE_SQL)
                .param(memberId)
                .query(rs -> {
                    active.put(rs.getString("order_id"), new ActivePickup(
                            rs.getString("pickup_id"), rs.getString("reservation_no"), rs.getString("status")));
                });
        return db.sql("SELECT order_id, order_no, ordered_at, business_status FROM orders"
                        + " WHERE member_id = ? AND business_status = 'CONFIRMED' ORDER BY ordered_at DESC, order_id")
                .param(memberId)
                .query((rs, n) -> {
                    String id = rs.getString("order_id");
                    return new Order(id, rs.getString("order_no"), ts(rs, "ordered_at"),
                            rs.getString("business_status"), items.getOrDefault(id, List.of()), active.get(id));
                })
                .list();
    }

    List<Location> locations() {
        return db.sql("SELECT location_id, name FROM pickup_locations WHERE is_active ORDER BY name, location_id")
                .query((rs, n) -> new Location(rs.getString(1), rs.getString(2)))
                .list();
    }

    private static Flight flight(ResultSet rs) throws SQLException {
        return new Flight(rs.getString("flight_id"), rs.getString("flight_no"),
                rs.getObject("service_date", LocalDate.class), ts(rs, "boarding_start"), ts(rs, "boarding_end"),
                ts(rs, "departure_at"), rs.getString("gate_code"));
    }

    List<Flight> flights(LocalDate serviceDate) {
        String order = " ORDER BY departure_at, flight_no";
        var spec = serviceDate == null
                ? db.sql("SELECT * FROM flights" + order)
                : db.sql("SELECT * FROM flights WHERE service_date = ?" + order).param(serviceDate);
        return spec.query((rs, n) -> flight(rs)).list();
    }

    /** D5: 현재+60분 이후, boarding_start 이전, UTC 기준 30분 정렬. 모든 위치 동일. */
    List<Instant> candidateTimes(Instant boardingStart) {
        Instant earliest = clock.instant().plus(MIN_LEAD);
        long slot = SLOT.toSeconds();
        long sec = earliest.getEpochSecond() + (earliest.getNano() > 0 ? 1 : 0);
        List<Instant> times = new ArrayList<>();
        for (Instant t = Instant.ofEpochSecond(Math.ceilDiv(sec, slot) * slot); t.isBefore(boardingStart); t = t.plus(SLOT)) {
            times.add(t);
        }
        return times;
    }

    /** 주문·위치·항공편을 확인하고 후보 시각을 돌려준다. 계약 외 추가: 예약 불가 주문은 409. */
    List<Instant> pickupTimes(String memberId, String orderId, String locationId, String flightId) {
        OrderRow order = order(memberId, orderId, false);
        requireReservable(order);
        requireLocation(locationId, order.siteId());
        return candidateTimes(flightRow(flightId).flight().boardingStart());
    }

    private OrderRow order(String memberId, String orderId, boolean lock) {
        return db.sql("SELECT site_id, business_status FROM orders WHERE order_id = ? AND member_id = ?"
                        + (lock ? " FOR UPDATE" : ""))
                .param(orderId).param(memberId)
                .query((rs, n) -> new OrderRow(rs.getString(1), rs.getString(2)))
                .optional()
                .orElseThrow(() -> ApiException.notFound("ORDER_NOT_FOUND"));
    }

    private static void requireReservable(OrderRow order) {
        if (!"CONFIRMED".equals(order.businessStatus())) {
            throw new ApiException(HttpStatus.CONFLICT, "ORDER_NOT_RESERVABLE", "예약할 수 없는 주문입니다.");
        }
    }

    private void requireLocation(String locationId, String siteId) {
        boolean ok = db.sql("SELECT 1 FROM pickup_locations WHERE location_id = ? AND site_id = ? AND is_active")
                .param(locationId).param(siteId).query().optionalValue().isPresent();
        if (!ok) {
            throw ApiException.notFound("LOCATION_NOT_FOUND");
        }
    }

    private record FlightRow(Flight flight, long revision) {}

    private FlightRow flightRow(String flightId) {
        return db.sql("SELECT * FROM flights WHERE flight_id = ?").param(flightId)
                .query((rs, n) -> new FlightRow(flight(rs), rs.getLong("revision")))
                .optional()
                .orElseThrow(() -> ApiException.notFound("FLIGHT_NOT_FOUND"));
    }

    /**
     * 같은 request_id가 있으면 저장본(같은 회원·같은 필드) 또는 409 IDEMPOTENCY_CONFLICT.
     * 잠금 순서는 orders → pickups. request_id UNIQUE 충돌 시 호출자가 이 메서드를 한 번 더 부른다.
     */
    @Transactional
    Created create(String memberId, CreatePickup req) {
        Optional<Pickup> replay = replay(memberId, req);
        if (replay.isPresent()) {
            return new Created(replay.get(), false);
        }
        OrderRow order = order(memberId, req.orderId(), true);
        replay = replay(memberId, req); // 같은 주문의 동시 재전송은 잠금 뒤에 보인다
        if (replay.isPresent()) {
            return new Created(replay.get(), false);
        }
        requireReservable(order);
        requireLocation(req.locationId(), order.siteId());
        FlightRow flight = flightRow(req.flightId());
        if (!candidateTimes(flight.flight().boardingStart()).contains(req.pickupAt())) {
            throw new ApiException(HttpStatus.UNPROCESSABLE_CONTENT, "PICKUP_TIME_NOT_AVAILABLE",
                    "선택할 수 없는 시각입니다.");
        }
        boolean hasActive = db.sql("SELECT 1 FROM pickups WHERE order_id = ? AND status IN " + ACTIVE_SQL)
                .param(req.orderId()).query().optionalValue().isPresent();
        if (hasActive) {
            throw new ApiException(HttpStatus.CONFLICT, "ACTIVE_PICKUP_EXISTS", "이미 진행 중인 예약이 있습니다.");
        }
        Map<String, Object> snapshot = json.convertValue(flight.flight(), new TypeReference<LinkedHashMap<String, Object>>() {});
        snapshot.put("revision", flight.revision());
        String pickupId = UUID.randomUUID().toString();
        for (int attempt = 0; ; attempt++) {
            int rows = db.sql("""
                    INSERT INTO pickups (pickup_id, request_id, reservation_no, order_id, location_id, flight_id,
                        flight_revision, flight_snapshot, pickup_at, channel, status, revision)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?::jsonb, ?, 'WEB', 'REQUESTED', 0)
                    ON CONFLICT (reservation_no) DO NOTHING""")
                    .param(pickupId).param(req.requestId()).param(reservationNo()).param(req.orderId())
                    .param(req.locationId()).param(req.flightId()).param(flight.revision())
                    .param(json.writeValueAsString(snapshot))
                    .param(OffsetDateTime.ofInstant(req.pickupAt(), ZoneOffset.UTC))
                    .update();
            if (rows == 1) {
                return new Created(pickup(memberId, pickupId).orElseThrow(), true);
            }
            if (attempt >= 10) {
                throw new IllegalStateException("reservation_no 발급 실패");
            }
        }
    }

    private String reservationNo() {
        StringBuilder sb = new StringBuilder(8);
        for (int i = 0; i < 8; i++) {
            sb.append(RESERVATION_ALPHABET.charAt(random.nextInt(RESERVATION_ALPHABET.length())));
        }
        return sb.toString();
    }

    private Optional<Pickup> replay(String memberId, CreatePickup req) {
        record Stored(String pickupId, String memberId, CreatePickup fields) {}
        return db.sql("""
                        SELECT p.pickup_id, o.member_id, p.request_id, p.order_id, p.location_id, p.flight_id, p.pickup_at
                        FROM pickups p JOIN orders o USING (order_id) WHERE p.request_id = ?""")
                .param(req.requestId())
                .query((rs, n) -> new Stored(rs.getString("pickup_id"), rs.getString("member_id"), new CreatePickup(
                        rs.getString("request_id"), rs.getString("order_id"), rs.getString("location_id"),
                        rs.getString("flight_id"), ts(rs, "pickup_at"))))
                .optional()
                .map(s -> {
                    if (!s.memberId().equals(memberId) || !s.fields().equals(req)) {
                        throw new ApiException(HttpStatus.CONFLICT, "IDEMPOTENCY_CONFLICT",
                                "같은 requestId로 다른 요청이 이미 처리되었습니다.");
                    }
                    return pickup(memberId, s.pickupId()).orElseThrow();
                });
    }

    private static final String PICKUP_SQL = """
            SELECT p.pickup_id, p.reservation_no, p.order_id, o.order_no, l.location_id, l.name,
                p.flight_snapshot::text AS flight_snapshot, p.pickup_at, p.status, p.revision, p.created_at
            FROM pickups p JOIN orders o USING (order_id) JOIN pickup_locations l USING (location_id)
            WHERE o.member_id = ?""";

    private Pickup mapPickup(ResultSet rs) throws SQLException {
        return new Pickup(rs.getString("pickup_id"), rs.getString("reservation_no"), rs.getString("order_id"),
                rs.getString("order_no"), new Location(rs.getString("location_id"), rs.getString("name")),
                json.readValue(rs.getString("flight_snapshot"), new TypeReference<LinkedHashMap<String, Object>>() {}),
                ts(rs, "pickup_at"), rs.getString("status"), rs.getLong("revision"), ts(rs, "created_at"));
    }

    List<Pickup> pickups(String memberId) {
        return db.sql(PICKUP_SQL + " ORDER BY p.created_at DESC, p.pickup_id").param(memberId)
                .query((rs, n) -> mapPickup(rs)).list();
    }

    Optional<Pickup> pickup(String memberId, String pickupId) {
        return db.sql(PICKUP_SQL + " AND p.pickup_id = ?").param(memberId).param(pickupId)
                .query((rs, n) -> mapPickup(rs)).optional();
    }
}
