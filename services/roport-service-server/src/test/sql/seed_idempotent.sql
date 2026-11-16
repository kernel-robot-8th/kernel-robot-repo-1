-- db/dev/seed.sql 재실행 안전성 검사. 격리 테스트 DB 전용, 가짜 값만 쓴다.
-- 실행: psql -v ON_ERROR_STOP=1 -d <격리 DB> -f src/test/sql/seed_idempotent.sql
-- 한 트랜잭션 안의 전용 스키마에서 돌리고 ROLLBACK 한다(남는 것 없음). 트랜잭션 안이라 now()가 고정돼 결과가 결정적이다.
\set ON_ERROR_STOP on
BEGIN;
CREATE SCHEMA seed_idempotent;
SET LOCAL search_path TO seed_idempotent;
\ir ../../main/resources/db/migration/V1__service_schema.sql
\ir ../../main/resources/db/dev/seed.sql

CREATE TEMP TABLE first_run AS SELECT flight_id, boarding_start, revision FROM flights;
-- 개발자가 손댄 기존 행은 재실행이 되돌리지 않아야 한다.
UPDATE members SET display_name = 'edited' WHERE member_id = 'dev-member-1';
UPDATE orders SET business_status = 'RECEIVED' WHERE order_id = 'dev-order-1';

-- 같은 시각 재실행: 행 수·revision·시각 불변
\ir ../../main/resources/db/dev/seed.sql
DO $$
BEGIN
    IF (SELECT count(*) FROM members) <> 2 OR (SELECT count(*) FROM orders) <> 5
            OR (SELECT count(*) FROM order_items) <> 6 OR (SELECT count(*) FROM pickup_locations) <> 4
            OR (SELECT count(*) FROM flights) <> (SELECT count(*) FROM first_run) THEN
        RAISE EXCEPTION 'seed 재실행이 행 수를 바꿨다';
    END IF;
    IF EXISTS (SELECT 1 FROM flights f JOIN first_run r USING (flight_id)
               WHERE f.revision <> r.revision OR f.boarding_start <> r.boarding_start) THEN
        RAISE EXCEPTION 'seed 같은 시각 재실행이 flights revision/시각을 바꿨다';
    END IF;
    IF (SELECT display_name FROM members WHERE member_id = 'dev-member-1') <> 'edited'
            OR (SELECT business_status FROM orders WHERE order_id = 'dev-order-1') <> 'RECEIVED' THEN
        RAISE EXCEPTION 'seed 재실행이 기존 회원/주문 행을 덮어썼다';
    END IF;
END $$;

-- 시각이 과거로 밀린 항공편은 실행 시각 기준으로 갱신되고 revision이 1 오른다.
UPDATE flights SET boarding_start = boarding_start - interval '2 hours', boarding_end = boarding_end - interval '2 hours',
    departure_at = departure_at - interval '2 hours' WHERE flight_id IN (SELECT flight_id FROM first_run LIMIT 1);
\ir ../../main/resources/db/dev/seed.sql
DO $$
BEGIN
    IF EXISTS (SELECT 1 FROM flights f JOIN first_run r USING (flight_id)
               WHERE f.boarding_start <> r.boarding_start
                  OR f.boarding_end <> r.boarding_start + interval '30 minutes'
                  OR f.departure_at <> r.boarding_start + interval '50 minutes') THEN
        RAISE EXCEPTION 'seed가 과거 시각 항공편을 갱신하지 않았다';
    END IF;
    IF (SELECT count(*) FROM flights f JOIN first_run r USING (flight_id) WHERE f.revision = r.revision + 1) <> 1
            OR (SELECT count(*) FROM flights f JOIN first_run r USING (flight_id) WHERE f.revision = r.revision)
               <> (SELECT count(*) - 1 FROM first_run) THEN
        RAISE EXCEPTION 'revision은 바뀐 항공편만 1 올라야 한다';
    END IF;
END $$;
ROLLBACK;
\echo 'seed 재실행 검사: 기대대로'
