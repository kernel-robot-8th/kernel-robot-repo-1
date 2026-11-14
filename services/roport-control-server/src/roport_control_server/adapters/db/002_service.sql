CREATE TABLE IF NOT EXISTS control_operational_state (
    singleton boolean PRIMARY KEY DEFAULT true CHECK (singleton),
    data jsonb NOT NULL DEFAULT '{}',
    updated_at timestamptz NOT NULL DEFAULT now()
);
INSERT INTO control_operational_state(singleton) VALUES(true) ON CONFLICT DO NOTHING;
CREATE TABLE IF NOT EXISTS control_service_tasks (
    task_id text PRIMARY KEY,
    site_id text NOT NULL,
    order_id text NOT NULL,
    reservation_id text NOT NULL UNIQUE,
    request jsonb NOT NULL,
    state text NOT NULL,
    revision bigint NOT NULL DEFAULT 0,
    progress jsonb NOT NULL DEFAULT '{}',
    updated_at timestamptz NOT NULL DEFAULT now()
);
CREATE UNIQUE INDEX IF NOT EXISTS control_one_active_order
ON control_service_tasks(order_id) WHERE state NOT IN ('COMPLETED','FAILED','REJECTED');
CREATE TABLE IF NOT EXISTS control_service_inbox (
    message_id text PRIMARY KEY,
    message jsonb NOT NULL,
    task_id text NOT NULL REFERENCES control_service_tasks
);
CREATE TABLE IF NOT EXISTS control_service_outbox (
    message_id text PRIMARY KEY,
    message jsonb NOT NULL,
    acknowledged_at timestamptz,
    created_at timestamptz NOT NULL DEFAULT now()
);
CREATE TABLE IF NOT EXISTS control_service_receipts (
    task_id text NOT NULL REFERENCES control_service_tasks,
    receipt_id text PRIMARY KEY,
    message jsonb NOT NULL,
    recorded_at timestamptz NOT NULL DEFAULT now()
);
-- Legacy receipts have no arrival evidence and must never authorize departure.
ALTER TABLE control_service_receipts
ADD COLUMN IF NOT EXISTS arrival_verified boolean NOT NULL DEFAULT false;
