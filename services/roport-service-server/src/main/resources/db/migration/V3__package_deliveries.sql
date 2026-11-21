-- One cart can deliver an order in several trips. Each receipt is immutable;
-- the order becomes RECEIVED only after all active packages have receipts.
ALTER TABLE receipts DROP CONSTRAINT receipts_order_id_key;
CREATE INDEX receipts_by_order ON receipts(order_id);
