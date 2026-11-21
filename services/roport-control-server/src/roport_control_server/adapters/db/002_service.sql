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
-- Preserve existing locks and leave pre-dispatch HOLD editable on every install.
ALTER TABLE control_service_tasks
ADD COLUMN IF NOT EXISTS dispatch_locked boolean NOT NULL DEFAULT false;
UPDATE control_service_tasks SET dispatch_locked=true
WHERE state IN ('RUNNING','COMPLETED','FAILED','REJECTED')
   OR EXISTS (SELECT 1 FROM control_operational_state s,
       jsonb_each(COALESCE(s.data->'tasks', '{}'::jsonb)) child
       WHERE starts_with(child.key, control_service_tasks.task_id || ':'));
-- Install sequence once; never rewind it on later migration runs.
DO $$ BEGIN
    IF NOT EXISTS (SELECT 1 FROM information_schema.columns
                   WHERE table_schema=current_schema() AND table_name='control_service_outbox' AND column_name='seq') THEN
        ALTER TABLE control_service_outbox ADD COLUMN seq bigserial;
        WITH ordered AS (
            SELECT message_id, row_number() OVER (
                ORDER BY message->>'task_id', (message->'payload'->>'task_revision')::bigint,
                         created_at, message_id) AS n FROM control_service_outbox
        )
        UPDATE control_service_outbox b SET seq=ordered.n FROM ordered WHERE b.message_id=ordered.message_id;
        PERFORM setval(pg_get_serial_sequence('control_service_outbox', 'seq'),
                       GREATEST(COALESCE((SELECT max(seq) FROM control_service_outbox), 0), 1));
    END IF;
END $$;
CREATE UNIQUE INDEX IF NOT EXISTS control_service_outbox_seq_idx ON control_service_outbox(seq);
CREATE TABLE IF NOT EXISTS control_service_quarantine (
    site_id text NOT NULL,
    message_id text NOT NULL,
    digest text NOT NULL,
    reason text NOT NULL,
    recorded_at timestamptz NOT NULL DEFAULT now(),
    PRIMARY KEY (site_id, message_id, digest)
);
