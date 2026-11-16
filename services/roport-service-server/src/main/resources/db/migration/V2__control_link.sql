-- Transport durability is separate from customer receipt/business status.
CREATE TABLE control_link_outbox (
    message_id text PRIMARY KEY,
    site_id text NOT NULL,
    source_ref text UNIQUE,
    message jsonb NOT NULL,
    acknowledged_at timestamptz,
    created_at timestamptz NOT NULL DEFAULT now()
);
CREATE TABLE control_link_inbox (
    message_id text PRIMARY KEY,
    site_id text NOT NULL,
    message jsonb NOT NULL,
    received_at timestamptz NOT NULL DEFAULT now()
);
