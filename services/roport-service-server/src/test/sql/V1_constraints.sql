-- V1__service_schema.sql 제약 검증(계약 5절 S1). 격리 테스트 DB 전용, 가짜 값만 쓴다.
-- 실행: psql -v ON_ERROR_STOP=1 -d <격리 DB> -f src/test/sql/V1_constraints.sql
-- 전용 스키마를 만들어 V1을 적용하고 끝나면 지운다(재실행 가능). 실패 케이스가 있으면 비제로 종료.
\set ON_ERROR_STOP on
SET client_min_messages = notice;
DROP SCHEMA IF EXISTS s1_v1_constraints CASCADE;
CREATE SCHEMA s1_v1_constraints;
SET search_path TO s1_v1_constraints;
\ir ../../main/resources/db/migration/V1__service_schema.sql

CREATE TABLE s1_failures (label text NOT NULL, detail text NOT NULL);

-- want NULL: 성공해야 함(결과 유지). want SQLSTATE: 그 코드로 거부돼야 함(23503 FK, 23505 UNIQUE, 23514 CHECK, P0001 trigger).
CREATE FUNCTION expect(label text, stmt text, want text) RETURNS void LANGUAGE plpgsql AS $$
DECLARE
    got text;
    msg text;
BEGIN
    BEGIN
        EXECUTE stmt;
    EXCEPTION WHEN others THEN
        got := SQLSTATE;
        msg := SQLERRM;
    END;
    IF want IS NULL AND got IS NULL THEN
        RAISE NOTICE '통과(정상): %', label;
    ELSIF want IS NULL THEN
        RAISE NOTICE 'FAIL 정상 케이스 거부됨: % [%] %', label, got, msg;
        INSERT INTO s1_failures VALUES (label, got || ' ' || msg);
    ELSIF got = want THEN
        RAISE NOTICE '거부됨 [%]: %', got, label;
    ELSIF got IS NULL THEN
        RAISE NOTICE 'FAIL 거부되지 않음: %', label;
        INSERT INTO s1_failures VALUES (label, 'not rejected');
    ELSE
        RAISE NOTICE 'FAIL 다른 이유로 거부됨: % (기대 %, 실제 %) %', label, want, got, msg;
        INSERT INTO s1_failures VALUES (label, got || ' ' || msg);
    END IF;
END $$;

-- 기본 데이터
SELECT expect('member', $q$INSERT INTO members (member_id) VALUES ('m1')$q$, NULL);
SELECT expect('orders o1·o2', $q$INSERT INTO orders (order_id, order_no, member_id, site_id, revision, business_status, ordered_at)
    VALUES ('o1','N1','m1','site-test',0,'CONFIRMED',now()), ('o2','N2','m1','site-test',0,'CONFIRMED',now())$q$, NULL);
SELECT expect('order_items', $q$INSERT INTO order_items (item_id, order_id, product_name, quantity)
    VALUES ('i1','o1','test-a',1), ('i2','o2','test-b',1), ('i3','o1','test-c',2)$q$, NULL);
SELECT expect('flight', $q$INSERT INTO flights VALUES ('f1','XX001','2026-01-01','2026-01-01 09:00Z','2026-01-01 09:30Z','2026-01-01 10:00Z','G1',0)$q$, NULL);
SELECT expect('pickup_location', $q$INSERT INTO pickup_locations VALUES ('l1','site-test','test-gate','cp-test',true)$q$, NULL);
SELECT expect('packages p1·p2·p3', $q$INSERT INTO packages (package_id, order_id, package_revision, label_ref)
    VALUES ('p1','o1',0,'lb1'), ('p2','o1',0,'lb2'), ('p3','o1',0,'lb3')$q$, NULL);
SELECT expect('package_items 같은 주문', $q$INSERT INTO package_items VALUES ('o1','p1','i1',1), ('o1','p2','i3',2)$q$, NULL);

-- 단순 CHECK
SELECT expect('flights 탑승 마감 > 출발', $q$INSERT INTO flights VALUES ('f2','XX002','2026-01-01','2026-01-01 09:00Z','2026-01-01 10:30Z','2026-01-01 10:00Z','G1',0)$q$, '23514');
SELECT expect('order_items quantity 0', $q$INSERT INTO order_items VALUES ('i9','o1','test',0)$q$, '23514');
SELECT expect('orders business_status 미정 값', $q$INSERT INTO orders (order_id, order_no, member_id, site_id, revision, business_status, ordered_at)
    VALUES ('o9','N9','m1','site-test',0,'SHIPPED',now())$q$, '23514');
SELECT expect('revision 음수', $q$UPDATE orders SET revision = -1 WHERE order_id = 'o1'$q$, '23514');

-- 다른 주문 item을 package에 넣기
SELECT expect('package_items: 다른 주문 item(order_id=o1)', $q$INSERT INTO package_items VALUES ('o1','p1','i2',1)$q$, '23503');
SELECT expect('package_items: 다른 주문 item(order_id=o2)', $q$INSERT INTO package_items VALUES ('o2','p1','i2',1)$q$, '23503');

-- 활성 예약 2건
SELECT expect('pickup pk1 REQUESTED', $q$INSERT INTO pickups (pickup_id, request_id, reservation_no, order_id, location_id, flight_id, flight_revision, flight_snapshot, pickup_at, channel, status, revision)
    VALUES ('pk1','rq1','R1','o1','l1','f1',0,'{}',now(),'test','REQUESTED',0)$q$, NULL);
SELECT expect('활성 예약 2건(o1 CONFIRMED)', $q$INSERT INTO pickups (pickup_id, request_id, reservation_no, order_id, location_id, flight_id, flight_revision, flight_snapshot, pickup_at, channel, status, revision)
    VALUES ('pk2','rq2','R2','o1','l1','f1',0,'{}',now(),'test','CONFIRMED',0)$q$, '23505');
SELECT expect('비활성 예약 추가(o1 CLOSED)', $q$INSERT INTO pickups (pickup_id, request_id, reservation_no, order_id, location_id, flight_id, flight_revision, flight_snapshot, pickup_at, channel, status, revision)
    VALUES ('pk3','rq3','R3','o1','l1','f1',0,'{}',now(),'test','CLOSED',0)$q$, NULL);

-- pending CHECK (o2는 활성 예약이 없어 CHECK만 걸린다)
SELECT expect('pending: CHANGE_PENDING change NULL', $q$INSERT INTO pickups (pickup_id, request_id, reservation_no, order_id, location_id, flight_id, flight_revision, flight_snapshot, pickup_at, channel, status, pending_request_id, revision)
    VALUES ('pk9','rq9','R9','o2','l1','f1',0,'{}',now(),'test','CHANGE_PENDING','pr9',0)$q$, '23514');
SELECT expect('pending: CANCEL_PENDING change NOT NULL', $q$INSERT INTO pickups (pickup_id, request_id, reservation_no, order_id, location_id, flight_id, flight_revision, flight_snapshot, pickup_at, channel, status, pending_request_id, pending_change, revision)
    VALUES ('pk9','rq9','R9','o2','l1','f1',0,'{}',now(),'test','CANCEL_PENDING','pr9','{}',0)$q$, '23514');
SELECT expect('pending: CANCEL_PENDING request NULL', $q$INSERT INTO pickups (pickup_id, request_id, reservation_no, order_id, location_id, flight_id, flight_revision, flight_snapshot, pickup_at, channel, status, revision)
    VALUES ('pk9','rq9','R9','o2','l1','f1',0,'{}',now(),'test','CANCEL_PENDING',0)$q$, '23514');
SELECT expect('pending: REQUESTED에 pending 값', $q$INSERT INTO pickups (pickup_id, request_id, reservation_no, order_id, location_id, flight_id, flight_revision, flight_snapshot, pickup_at, channel, status, pending_request_id, revision)
    VALUES ('pk9','rq9','R9','o2','l1','f1',0,'{}',now(),'test','REQUESTED','pr9',0)$q$, '23514');
SELECT expect('pending: CHANGE_PENDING 둘 다 NN', $q$INSERT INTO pickups (pickup_id, request_id, reservation_no, order_id, location_id, flight_id, flight_revision, flight_snapshot, pickup_at, channel, status, pending_request_id, pending_change, revision)
    VALUES ('pk4','rq4','R4','o2','l1','f1',0,'{}',now(),'test','CHANGE_PENDING','pr4','{}',0)$q$, NULL);

-- num_nonnulls
SELECT expect('qr: 소유자 0개', $q$INSERT INTO qr_credentials (credential_id, token_hash, issued_at) VALUES ('c9','th9',now())$q$, '23514');
SELECT expect('qr: 소유자 2개', $q$INSERT INTO qr_credentials (credential_id, token_hash, member_id, pickup_id, issued_at) VALUES ('c9','th9','m1','pk1',now())$q$, '23514');
SELECT expect('qr: 소유자 1개', $q$INSERT INTO qr_credentials (credential_id, token_hash, pickup_id, issued_at) VALUES ('c1','th1','pk1',now())$q$, NULL);

-- marker
SELECT expect('marker 34', $q$INSERT INTO verifications (verification_id, verification_request_id, pickup_id, method, result, actor_ref, marker_id) VALUES ('v9','vr9','pk1','test','test','actor-test',34)$q$, '23514');
SELECT expect('marker 50', $q$INSERT INTO verifications (verification_id, verification_request_id, pickup_id, method, result, actor_ref, marker_id) VALUES ('v9','vr9','pk1','test','test','actor-test',50)$q$, '23514');
SELECT expect('verification v1 marker 35', $q$INSERT INTO verifications (verification_id, verification_request_id, pickup_id, method, result, actor_ref, credential_id, marker_id) VALUES ('v1','vr1','pk1','test','test','actor-test','c1',35)$q$, NULL);
SELECT expect('verification v2(pk3) marker NULL', $q$INSERT INTO verifications (verification_id, verification_request_id, pickup_id, method, result, actor_ref) VALUES ('v2','vr2','pk3','test','test','actor-test')$q$, NULL);
SELECT expect('marker 최초 결속(NULL->49)', $q$UPDATE verifications SET marker_id = 49 WHERE verification_id = 'v2'$q$, NULL);
SELECT expect('marker 결속 후 변경', $q$UPDATE verifications SET marker_id = 40 WHERE verification_id = 'v1'$q$, 'P0001');
SELECT expect('marker 결속 후 NULL화', $q$UPDATE verifications SET marker_id = NULL WHERE verification_id = 'v1'$q$, 'P0001');

-- approval pickup·order 불일치
SELECT expect('approval: pickup·order 불일치(pk1,o2)', $q$INSERT INTO handover_approvals (approval_id, pickup_id, order_id, verification_id, task_ref, cart_ref, order_revision, manifest_hash)
    VALUES ('a9','pk1','o2','v1','t1','cart1',0,'mh')$q$, '23503');
SELECT expect('approval: 다른 pickup의 verification', $q$INSERT INTO handover_approvals (approval_id, pickup_id, order_id, verification_id, task_ref, cart_ref, order_revision, manifest_hash)
    VALUES ('a9','pk1','o1','v2','t1','cart1',0,'mh')$q$, '23503');
SELECT expect('approval a1', $q$INSERT INTO handover_approvals (approval_id, pickup_id, order_id, verification_id, task_ref, cart_ref, order_revision, manifest_hash)
    VALUES ('a1','pk1','o1','v1','t1','cart1',0,'mh')$q$, NULL);
SELECT expect('활성 approval 2건', $q$INSERT INTO handover_approvals (approval_id, pickup_id, order_id, verification_id, task_ref, cart_ref, order_revision, manifest_hash)
    VALUES ('a2','pk1','o1','v1','t1','cart1',0,'mh')$q$, '23505');

-- 잠긴 manifest
SELECT expect('p2 manifest 잠금', $q$UPDATE packages SET manifest_locked_at = now() WHERE package_id = 'p2'$q$, NULL);
SELECT expect('잠긴 manifest INSERT', $q$INSERT INTO package_items VALUES ('o1','p2','i1',1)$q$, 'P0001');
SELECT expect('잠긴 manifest UPDATE 수량', $q$UPDATE package_items SET quantity = 1 WHERE package_id = 'p2'$q$, 'P0001');
SELECT expect('잠긴 manifest UPDATE 부모 이동(p2->p3)', $q$UPDATE package_items SET package_id = 'p3' WHERE package_id = 'p2'$q$, 'P0001');
SELECT expect('잠긴 manifest UPDATE 부모 이동(p1->p2)', $q$UPDATE package_items SET package_id = 'p2' WHERE package_id = 'p1'$q$, 'P0001');
SELECT expect('잠긴 manifest DELETE', $q$DELETE FROM package_items WHERE package_id = 'p2'$q$, 'P0001');
SELECT expect('잠긴 package 잠금 해제(NULL)', $q$UPDATE packages SET manifest_locked_at = NULL WHERE package_id = 'p2'$q$, 'P0001');
SELECT expect('잠긴 package 잠금 시각 변경', $q$UPDATE packages SET manifest_locked_at = now() + interval '1 s' WHERE package_id = 'p2'$q$, 'P0001');
SELECT expect('잠긴 package order_id 변경', $q$UPDATE packages SET order_id = 'o2' WHERE package_id = 'p2'$q$, 'P0001');
SELECT expect('잠긴 package DELETE', $q$DELETE FROM packages WHERE package_id = 'p2'$q$, 'P0001');
SELECT expect('잠금 전 package 간 이동(p1->p3)', $q$UPDATE package_items SET package_id = 'p3' WHERE package_id = 'p1'$q$, NULL);
SELECT expect('잠긴 package label 변경(계약상 허용)', $q$UPDATE packages SET label_ref = 'lb2b' WHERE package_id = 'p2'$q$, NULL);

-- receipt
SELECT expect('receipt approval 조합 불일치(pk3)', $q$INSERT INTO receipts (receipt_id, order_id, pickup_id, approval_id, confirmation_request_id, confirmed_by, channel, received_at, order_revision)
    VALUES ('r3','o1','pk3','a1','cr3','actor-test','test',now(),0)$q$, '23503');
SELECT expect('receipt r1', $q$INSERT INTO receipts (receipt_id, order_id, pickup_id, approval_id, confirmation_request_id, confirmed_by, channel, received_at, order_revision)
    VALUES ('r1','o1','pk1','a1','cr1','actor-test','test',now(),0)$q$, NULL);
SELECT expect('중복 receipt(같은 주문·approval)', $q$INSERT INTO receipts (receipt_id, order_id, pickup_id, approval_id, confirmation_request_id, confirmed_by, channel, received_at, order_revision)
    VALUES ('r2','o1','pk1','a1','cr2','actor-test','test',now(),0)$q$, '23505');
SELECT expect('receipt_packages r1·p2', $q$INSERT INTO receipt_packages VALUES ('r1','o1','p2',0)$q$, NULL);
SELECT expect('중복 package 수령', $q$INSERT INTO receipt_packages VALUES ('r1','o1','p2',0)$q$, '23505');
SELECT expect('receipt_packages 다른 주문 package', $q$INSERT INTO receipt_packages VALUES ('r1','o2','p3',0)$q$, '23503');

-- customer_service_requests
SELECT expect('csr1 REQUESTED', $q$INSERT INTO customer_service_requests (request_id, pickup_id, verification_id, mode, expected_pickup_revision, status, requested_at)
    VALUES ('s1','pk1','v1','test',0,'REQUESTED',now())$q$, NULL);
SELECT expect('활성 service request 2건', $q$INSERT INTO customer_service_requests (request_id, pickup_id, verification_id, mode, expected_pickup_revision, status, requested_at)
    VALUES ('s2','pk1','v1','test',0,'SELECTED',now())$q$, '23505');
SELECT expect('csr PENDING_CHANGE supersedes s1', $q$INSERT INTO customer_service_requests (request_id, pickup_id, verification_id, mode, expected_pickup_revision, supersedes_request_id, status, requested_at)
    VALUES ('s3','pk1','v1','test',0,'s1','PENDING_CHANGE',now())$q$, NULL);
SELECT expect('PENDING_CHANGE 2건', $q$INSERT INTO customer_service_requests (request_id, pickup_id, verification_id, mode, expected_pickup_revision, status, requested_at)
    VALUES ('s4','pk1','v1','test',0,'PENDING_CHANGE',now())$q$, '23505');
SELECT expect('같은 요청 두 번 대체', $q$INSERT INTO customer_service_requests (request_id, pickup_id, verification_id, mode, expected_pickup_revision, supersedes_request_id, status, requested_at)
    VALUES ('s5','pk1','v1','test',0,'s1','DONE',now())$q$, '23505');
SELECT expect('자기 자신 대체', $q$INSERT INTO customer_service_requests (request_id, pickup_id, verification_id, mode, expected_pickup_revision, supersedes_request_id, status, requested_at)
    VALUES ('s6','pk1','v1','test',0,'s6','DONE',now())$q$, '23514');
SELECT expect('다른 pickup 요청 대체', $q$INSERT INTO customer_service_requests (request_id, pickup_id, verification_id, mode, expected_pickup_revision, supersedes_request_id, status, requested_at)
    VALUES ('s7','pk3','v2','test',0,'s3','DONE',now())$q$, '23503');

SELECT count(*) = 0 AS s1_ok, count(*) AS s1_fail FROM s1_failures \gset
TABLE s1_failures;
RESET search_path;
DROP SCHEMA s1_v1_constraints CASCADE;
\if :s1_ok
\echo 'S1 제약 검증: 모든 케이스 기대대로'
\else
\echo 'S1 제약 검증 실패:' :s1_fail
DO $$ BEGIN RAISE EXCEPTION 'S1 constraint verification failed'; END $$;
\endif
