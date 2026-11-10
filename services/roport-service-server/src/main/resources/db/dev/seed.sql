-- 개발용 더미 데이터(가짜 값만). Flyway 경로 밖이며 자동 적용되지 않는다. 운영 DB에 넣지 않는다.
-- 실행: psql -v ON_ERROR_STOP=1 -d <개발 DB> -f src/main/resources/db/dev/seed.sql (V1 마이그레이션 적용 후)
-- 재실행해도 기존 행은 그대로 둔다(항공편 시각만 갱신). 항공편은 실행 시각 기준 앞으로의 시간으로 만든다.
-- 로그인: login_id 'demo1' 또는 'demo2' (개발 전용, 비밀번호 없음)

INSERT INTO members (member_id, login_id, display_name) VALUES
    ('dev-member-1', 'demo1', '테스트 고객 1'),
    ('dev-member-2', 'demo2', '테스트 고객 2')
ON CONFLICT DO NOTHING;

INSERT INTO pickup_locations (location_id, site_id, name, control_point_ref, is_active) VALUES
    ('dev-loc-gate1', 'DEV-SITE', 'Gate 1', 'DEV-GATE-1', true),
    ('dev-loc-gate2', 'DEV-SITE', 'Gate 2', 'DEV-GATE-2', true),
    ('dev-loc-gate3', 'DEV-SITE', 'Gate 3', 'DEV-GATE-3', true),
    ('dev-loc-arrival', 'DEV-SITE', '입국장', 'DEV-ARRIVAL', true)
ON CONFLICT DO NOTHING;

INSERT INTO orders (order_id, order_no, member_id, site_id, revision, business_status, ordered_at) VALUES
    ('dev-order-1', 'DEV-0001', 'dev-member-1', 'DEV-SITE', 0, 'CONFIRMED', now() - interval '2 days'),
    ('dev-order-2', 'DEV-0002', 'dev-member-1', 'DEV-SITE', 0, 'CONFIRMED', now() - interval '1 day'),
    ('dev-order-3', 'DEV-0003', 'dev-member-1', 'DEV-SITE', 0, 'RECEIVED', now() - interval '5 days'),
    ('dev-order-4', 'DEV-0004', 'dev-member-1', 'DEV-SITE', 0, 'CANCELLED', now() - interval '4 days'),
    ('dev-order-5', 'DEV-0005', 'dev-member-2', 'DEV-SITE', 0, 'CONFIRMED', now() - interval '1 day')
ON CONFLICT DO NOTHING;

INSERT INTO order_items (item_id, order_id, product_name, quantity) VALUES
    ('dev-item-1', 'dev-order-1', '샘플 향수 A', 1),
    ('dev-item-2', 'dev-order-1', '샘플 화장품 B', 2),
    ('dev-item-3', 'dev-order-2', '샘플 초콜릿 C', 3),
    ('dev-item-4', 'dev-order-3', '샘플 시계 D', 1),
    ('dev-item-5', 'dev-order-4', '샘플 가방 E', 1),
    ('dev-item-6', 'dev-order-5', '샘플 주류 F', 2)
ON CONFLICT DO NOTHING;

-- 탑승 시작 = 현재 정시 + N시간, 탑승 마감 +30분, 출발 +50분. service_date는 출발 시각의 Asia/Seoul 날짜.
INSERT INTO flights (flight_id, flight_no, service_date, boarding_start, boarding_end, departure_at, gate_code, revision)
SELECT 'dev-flight-' || f.no || '-' || d, f.no, d, b, b + interval '30 minutes', b + interval '50 minutes', f.gate, 0
FROM (VALUES ('ZZ101', 3, 'G1'), ('ZZ202', 5, 'G2'), ('ZZ303', 8, 'G3'), ('ZZ404', 26, 'G1')) AS f(no, hours, gate),
     LATERAL (SELECT date_trunc('hour', now()) + make_interval(hours => f.hours) AS b) AS t,
     LATERAL (SELECT ((b + interval '50 minutes') AT TIME ZONE 'Asia/Seoul')::date AS d) AS s
-- 같은 날 다른 시각에 재실행하면 시각을 실행 시각 기준으로 옮기고 revision을 올린다(기존 예약은 flight_snapshot을 유지).
ON CONFLICT (flight_id) DO UPDATE SET
    boarding_start = EXCLUDED.boarding_start,
    boarding_end   = EXCLUDED.boarding_end,
    departure_at   = EXCLUDED.departure_at,
    revision       = flights.revision + 1,
    updated_at     = now()
WHERE flights.boarding_start IS DISTINCT FROM EXCLUDED.boarding_start;
