ALTER TABLE receipts ADD COLUMN follow_card integer CHECK (follow_card BETWEEN 35 AND 49);
