-- RO:PORT 서비스 업무 DB v1 (제안, 팀 승인 전). PostgreSQL 16.
-- 서비스 서버 시작 시 Flyway가 이 파일을 적용한다. 공유 DB에 적용한 뒤에는 이 파일을 수정하지 않고 V2 이후로만 변경한다(체크섬).
-- 설계 원본: docs/database/README.md 4·6절. 이 파일은 설계의 제안 구현이며 확정본이 아니다.
-- 값이 정해지지 않은 enum(상태·방법·결과·채널 등)은 NOT NULL만 두고 CHECK를 넣지 않는다.
-- 임시 타입: 해시류 text, revision bigint(>=0, 초기값 미정이라 DEFAULT 없음), ID text(길이 상한 미정).
-- updated_at 갱신은 앱 책임. FK는 기본 RESTRICT(NO ACTION), cascade 없음. seed 없음.
-- 고객 서비스 요청(customer_service_requests)과 추종 마커 컬럼은 시나리오 확정 후 V2로 추가한다.

CREATE TABLE members (
    member_id     text PRIMARY KEY,
    login_id      text UNIQUE,          -- NULL 다수 허용(QR 전용 회원)
    password_hash text,
    display_name  text,
    -- 연락 정보 컬럼 미정
    created_at    timestamptz NOT NULL DEFAULT now(),
    updated_at    timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE admins (
    admin_id      text PRIMARY KEY,
    login_id      text NOT NULL UNIQUE,
    password_hash text NOT NULL,
    is_active     boolean NOT NULL,     -- 기본값 미정
    role          text NOT NULL,        -- 값 미정
    created_at    timestamptz NOT NULL DEFAULT now(),
    updated_at    timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE flights (
    flight_id      text PRIMARY KEY,
    flight_no      text NOT NULL,
    service_date   date NOT NULL,       -- 시간대 기준 미정
    boarding_start timestamptz NOT NULL,
    boarding_end   timestamptz NOT NULL,
    departure_at   timestamptz NOT NULL,
    gate_code      text NOT NULL,
    revision       bigint NOT NULL CHECK (revision >= 0),
    created_at     timestamptz NOT NULL DEFAULT now(),
    updated_at     timestamptz NOT NULL DEFAULT now(),
    UNIQUE (flight_no, service_date),
    CHECK (boarding_start <= boarding_end AND boarding_end <= departure_at)
);

CREATE TABLE pickup_locations (
    location_id       text PRIMARY KEY,
    site_id           text NOT NULL,
    name              text NOT NULL,
    control_point_ref text NOT NULL,
    is_active         boolean NOT NULL,
    created_at        timestamptz NOT NULL DEFAULT now(),
    updated_at        timestamptz NOT NULL DEFAULT now(),
    UNIQUE (site_id, control_point_ref)
);

CREATE TABLE orders (
    order_id        text PRIMARY KEY,
    order_no        text NOT NULL UNIQUE,
    member_id       text NOT NULL REFERENCES members,
    site_id         text NOT NULL,
    revision        bigint NOT NULL CHECK (revision >= 0),
    business_status text NOT NULL CHECK (business_status IN ('CONFIRMED', 'RECEIVED', 'CANCELLED')),
    ordered_at      timestamptz NOT NULL,
    created_at      timestamptz NOT NULL DEFAULT now(),
    updated_at      timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE order_items (
    item_id      text PRIMARY KEY,
    order_id     text NOT NULL REFERENCES orders,
    product_name text NOT NULL,
    quantity     integer NOT NULL CHECK (quantity > 0),
    created_at   timestamptz NOT NULL DEFAULT now(),
    updated_at   timestamptz NOT NULL DEFAULT now(),
    UNIQUE (order_id, item_id)
);

CREATE TABLE packages (
    package_id         text PRIMARY KEY,
    order_id           text NOT NULL REFERENCES orders,
    package_revision   bigint NOT NULL CHECK (package_revision >= 0),
    label_ref          text NOT NULL,
    manifest_locked_at timestamptz,     -- 잠금은 되돌릴 수 없다. 재포장은 새 package_id
    inactive_at        timestamptz,     -- 재포장·폐기로 비활성화된 시각. 행은 지우지 않는다(잠긴 package도 채울 수 있음)
    replaced_by_package_id text,        -- 대체한 같은 주문의 새 package. 폐기(대체 없음)면 NULL
    created_at         timestamptz NOT NULL DEFAULT now(),
    updated_at         timestamptz NOT NULL DEFAULT now(),
    UNIQUE (order_id, package_id),
    FOREIGN KEY (order_id, replaced_by_package_id) REFERENCES packages (order_id, package_id),
    CHECK (replaced_by_package_id <> package_id),
    CHECK (replaced_by_package_id IS NULL OR inactive_at IS NOT NULL)
);

CREATE TABLE package_items (
    order_id   text NOT NULL,
    package_id text NOT NULL,
    item_id    text NOT NULL,
    quantity   integer NOT NULL CHECK (quantity > 0),
    created_at timestamptz NOT NULL DEFAULT now(),
    updated_at timestamptz NOT NULL DEFAULT now(),
    PRIMARY KEY (package_id, item_id),
    FOREIGN KEY (order_id, package_id) REFERENCES packages (order_id, package_id),
    FOREIGN KEY (order_id, item_id) REFERENCES order_items (order_id, item_id)
);

CREATE FUNCTION packages_manifest_lock_guard() RETURNS trigger
LANGUAGE plpgsql SET search_path FROM CURRENT AS $$
BEGIN
    IF OLD.manifest_locked_at IS NOT NULL THEN
        IF TG_OP = 'DELETE' THEN
            RAISE EXCEPTION 'package % manifest is locked: delete rejected', OLD.package_id;
        END IF;
        IF NEW.manifest_locked_at IS DISTINCT FROM OLD.manifest_locked_at
           OR NEW.order_id IS DISTINCT FROM OLD.order_id THEN
            RAISE EXCEPTION 'package % manifest is locked: lock/order change rejected', OLD.package_id;
        END IF;
    END IF;
    RETURN CASE WHEN TG_OP = 'DELETE' THEN OLD ELSE NEW END;
END $$;

CREATE TRIGGER packages_manifest_lock_guard
BEFORE UPDATE OR DELETE ON packages
FOR EACH ROW EXECUTE FUNCTION packages_manifest_lock_guard();

-- 부모 package를 FOR SHARE로 잠가 동시 잠금(UPDATE)과 직렬화한다. FK의 KEY SHARE로는 막히지 않는다.
CREATE FUNCTION package_items_manifest_lock_guard() RETURNS trigger
LANGUAGE plpgsql SET search_path FROM CURRENT AS $$
DECLARE
    pkg       text;
    locked_at timestamptz;
BEGIN
    FOREACH pkg IN ARRAY CASE TG_OP
        WHEN 'INSERT' THEN ARRAY[NEW.package_id]
        WHEN 'DELETE' THEN ARRAY[OLD.package_id]
        ELSE ARRAY[OLD.package_id, NEW.package_id] END
    LOOP
        SELECT manifest_locked_at INTO locked_at FROM packages WHERE package_id = pkg FOR SHARE;
        IF locked_at IS NOT NULL THEN
            RAISE EXCEPTION 'package % manifest is locked: package_items % rejected', pkg, TG_OP;
        END IF;
    END LOOP;
    RETURN CASE WHEN TG_OP = 'DELETE' THEN OLD ELSE NEW END;
END $$;

CREATE TRIGGER package_items_manifest_lock_guard
BEFORE INSERT OR UPDATE OR DELETE ON package_items
FOR EACH ROW EXECUTE FUNCTION package_items_manifest_lock_guard();

CREATE TABLE pickups (
    pickup_id          text PRIMARY KEY,
    request_id         text NOT NULL UNIQUE,
    reservation_no     text NOT NULL UNIQUE,
    order_id           text NOT NULL REFERENCES orders,
    location_id        text NOT NULL REFERENCES pickup_locations,
    flight_id          text NOT NULL REFERENCES flights,
    flight_revision    bigint NOT NULL CHECK (flight_revision >= 0),
    flight_snapshot    jsonb NOT NULL,
    pickup_at          timestamptz NOT NULL,
    channel            text NOT NULL,   -- 값 미정
    status             text NOT NULL,   -- 전체 값 미정(아래 4개만 계약에 명시)
    pending_request_id text UNIQUE,
    pending_change     jsonb,
    revision           bigint NOT NULL CHECK (revision >= 0),
    created_at         timestamptz NOT NULL DEFAULT now(),
    updated_at         timestamptz NOT NULL DEFAULT now(),
    UNIQUE (pickup_id, order_id),
    CHECK (CASE status
        WHEN 'CHANGE_PENDING' THEN pending_request_id IS NOT NULL AND pending_change IS NOT NULL
        WHEN 'CANCEL_PENDING' THEN pending_request_id IS NOT NULL AND pending_change IS NULL
        ELSE pending_request_id IS NULL AND pending_change IS NULL END)
);

CREATE UNIQUE INDEX pickups_one_active_per_order ON pickups (order_id)
    WHERE status IN ('REQUESTED', 'CONFIRMED', 'CHANGE_PENDING', 'CANCEL_PENDING');

CREATE TABLE qr_credentials (
    credential_id text PRIMARY KEY,
    token_hash    text NOT NULL UNIQUE,
    member_id     text REFERENCES members,
    order_id      text REFERENCES orders,
    pickup_id     text REFERENCES pickups,
    issued_at     timestamptz NOT NULL,
    expires_at    timestamptz,          -- NULL 의미(무기한/미정) 미정
    revoked_at    timestamptz,
    created_at    timestamptz NOT NULL DEFAULT now(),
    updated_at    timestamptz NOT NULL DEFAULT now(),
    CHECK (num_nonnulls(member_id, order_id, pickup_id) = 1)
);

CREATE TABLE verifications (
    verification_id         text PRIMARY KEY,
    verification_request_id text NOT NULL UNIQUE,
    pickup_id               text NOT NULL REFERENCES pickups,
    method                  text NOT NULL,  -- 값 미정
    result                  text NOT NULL,  -- 값 미정
    verified_at             timestamptz,
    actor_ref               text NOT NULL,
    credential_id           text REFERENCES qr_credentials,
    reason_code             text,
    created_at              timestamptz NOT NULL DEFAULT now(),
    updated_at              timestamptz NOT NULL DEFAULT now(),
    UNIQUE (verification_id, pickup_id)
);

CREATE TABLE handover_approvals (
    approval_id     text PRIMARY KEY,
    pickup_id       text NOT NULL,
    order_id        text NOT NULL,
    verification_id text NOT NULL,
    task_ref        text NOT NULL,
    cart_ref        text NOT NULL,
    order_revision  bigint NOT NULL CHECK (order_revision >= 0),
    manifest_hash   text NOT NULL,
    expires_at      timestamptz,
    revoked_at      timestamptz,
    consumed_at     timestamptz,
    created_at      timestamptz NOT NULL DEFAULT now(),
    updated_at      timestamptz NOT NULL DEFAULT now(),
    UNIQUE (approval_id, order_id, pickup_id),
    FOREIGN KEY (pickup_id, order_id) REFERENCES pickups (pickup_id, order_id),
    FOREIGN KEY (verification_id, pickup_id) REFERENCES verifications (verification_id, pickup_id)
);

-- 만료 행은 새 승인 전에 같은 트랜잭션에서 revoked_at을 채운다(앱 책임).
CREATE UNIQUE INDEX handover_approvals_one_active_per_pickup ON handover_approvals (pickup_id)
    WHERE consumed_at IS NULL AND revoked_at IS NULL;

CREATE TABLE receipts (
    receipt_id              text PRIMARY KEY,
    order_id                text NOT NULL UNIQUE,
    pickup_id               text NOT NULL,
    approval_id             text NOT NULL UNIQUE,
    confirmation_request_id text NOT NULL UNIQUE,
    confirmed_by            text NOT NULL,
    channel                 text NOT NULL,
    received_at             timestamptz NOT NULL,
    order_revision          bigint NOT NULL CHECK (order_revision >= 0),
    created_at              timestamptz NOT NULL DEFAULT now(),
    updated_at              timestamptz NOT NULL DEFAULT now(),
    UNIQUE (receipt_id, order_id),      -- README 누락, receipt_packages FK 대상
    FOREIGN KEY (approval_id, order_id, pickup_id)
        REFERENCES handover_approvals (approval_id, order_id, pickup_id)
);

CREATE TABLE receipt_packages (
    receipt_id       text NOT NULL,
    order_id         text NOT NULL,
    package_id       text NOT NULL UNIQUE,
    package_revision bigint NOT NULL CHECK (package_revision >= 0),
    created_at       timestamptz NOT NULL DEFAULT now(),
    updated_at       timestamptz NOT NULL DEFAULT now(),
    PRIMARY KEY (receipt_id, package_id),
    FOREIGN KEY (receipt_id, order_id) REFERENCES receipts (receipt_id, order_id),
    FOREIGN KEY (order_id, package_id) REFERENCES packages (order_id, package_id)
);

CREATE TABLE notifications (
    notification_id text PRIMARY KEY,
    pickup_id       text NOT NULL REFERENCES pickups,
    event_key       text NOT NULL UNIQUE,
    type            text NOT NULL,      -- 값 미정
    delivery_status text NOT NULL,      -- 값 미정
    sent_at         timestamptz,
    task_ref        text,
    robot_ref       text,
    created_at      timestamptz NOT NULL DEFAULT now(),
    updated_at      timestamptz NOT NULL DEFAULT now()
);

-- 관제 상태의 읽기 사본. 낡은 revision 거부는 조건부 UPDATE(앱 책임).
CREATE TABLE control_task_views (
    task_ref         text PRIMARY KEY,
    pickup_id        text NOT NULL REFERENCES pickups,
    site_id          text NOT NULL,
    task_revision    bigint NOT NULL CHECK (task_revision >= 0),
    state            text NOT NULL,     -- 값 미정
    progress_payload jsonb NOT NULL,
    observed_at      timestamptz NOT NULL,
    synced_at        timestamptz NOT NULL,
    created_at       timestamptz NOT NULL DEFAULT now(),
    updated_at       timestamptz NOT NULL DEFAULT now()
);

-- HTTP 재전송 결과 저장. 업무 변경과 같은 트랜잭션에서 commit(앱 책임).
CREATE TABLE request_results (
    site_id                text NOT NULL,
    actor_type             text NOT NULL,  -- 값 미정
    actor_id               text NOT NULL,
    request_id             text NOT NULL,
    request_type           text NOT NULL,  -- 값 미정
    canonical_payload_hash text NOT NULL,
    target_ref             text,
    result_payload         jsonb NOT NULL,
    applied_at             timestamptz NOT NULL,
    created_at             timestamptz NOT NULL DEFAULT now(),
    updated_at             timestamptz NOT NULL DEFAULT now(),
    PRIMARY KEY (site_id, actor_type, actor_id, request_id)
);
