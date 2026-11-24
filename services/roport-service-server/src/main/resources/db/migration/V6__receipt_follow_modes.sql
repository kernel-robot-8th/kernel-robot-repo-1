-- NULL preserves the old card-only wire payload for existing clients and receipts.
ALTER TABLE receipts ADD COLUMN follow_mode text CHECK (follow_mode IN ('none', 'marker', 'appearance'));
ALTER TABLE receipts ADD CONSTRAINT receipt_follow_target CHECK (
    follow_mode IS NULL
    OR (follow_mode = 'marker' AND follow_card IS NOT NULL)
    OR (follow_mode IN ('none', 'appearance') AND follow_card IS NULL)
);
