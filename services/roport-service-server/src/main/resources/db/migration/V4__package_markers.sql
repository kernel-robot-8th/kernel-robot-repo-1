-- A package keeps its historical marker even after the empty box is returned.
CREATE TABLE package_markers (
    package_id text PRIMARY KEY REFERENCES packages(package_id),
    marker text NOT NULL CHECK (marker IN ('15','16','17','19','20','21','22')),
    assigned_at timestamptz NOT NULL DEFAULT now(),
    empty_box_returned_at timestamptz CHECK (empty_box_returned_at >= assigned_at)
);

CREATE UNIQUE INDEX one_package_per_active_marker
    ON package_markers(marker) WHERE empty_box_returned_at IS NULL;
