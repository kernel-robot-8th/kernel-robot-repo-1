CREATE TABLE control_registry_meta (
    singleton boolean PRIMARY KEY DEFAULT true CHECK (singleton),
    schema_version integer NOT NULL CHECK (schema_version = 1),
    registry_id uuid NOT NULL,
    development boolean NOT NULL DEFAULT false,
    revision bigint NOT NULL DEFAULT 0 CHECK (revision >= 0)
);
CREATE TABLE control_robot_slots (
    robot_id text PRIMARY KEY CHECK (robot_id IN ('pinky1','pinky2','pinky3','omx1','omx2','omx3')),
    revision bigint NOT NULL DEFAULT 0 CHECK (revision >= 0),
    kind text NOT NULL CHECK (kind IN ('pinky','omx')),
    host_id text NOT NULL,
    arm_id text
);
INSERT INTO control_robot_slots (robot_id, kind, host_id, arm_id) VALUES
    ('pinky1','pinky','pinky1',NULL), ('pinky2','pinky','pinky2',NULL),
    ('pinky3','pinky','pinky3',NULL), ('omx1','omx','shelf-01','omx1'),
    ('omx2','omx','shelf-01','omx2'), ('omx3','omx','omx3','omx3');
CREATE TABLE control_robot_bindings (
    binding_id uuid PRIMARY KEY,
    robot_id text NOT NULL REFERENCES control_robot_slots(robot_id) CHECK (robot_id IN ('pinky1','pinky2','pinky3')),
    device_id text NOT NULL CHECK (length(device_id) BETWEEN 1 AND 64),
    ip inet NOT NULL CHECK (family(ip) = 4 AND masklen(ip) = 32),
    role text NOT NULL CHECK (role IN ('logistics','smartcart')),
    ros_domain_id integer NOT NULL CHECK (ros_domain_id BETWEEN 0 AND 101),
    calibration text NOT NULL,
    desired_revision integer NOT NULL DEFAULT 1 CHECK (desired_revision > 0),
    excluded boolean NOT NULL DEFAULT false,
    created_at timestamptz NOT NULL DEFAULT now(),
    closed_at timestamptz
);
CREATE UNIQUE INDEX control_binding_slot ON control_robot_bindings(robot_id) WHERE closed_at IS NULL;
CREATE UNIQUE INDEX control_binding_device ON control_robot_bindings(device_id) WHERE closed_at IS NULL;
CREATE UNIQUE INDEX control_binding_domain ON control_robot_bindings(ros_domain_id) WHERE closed_at IS NULL;
CREATE UNIQUE INDEX control_binding_ip ON control_robot_bindings(ip) WHERE closed_at IS NULL;
CREATE TABLE control_registry_changes (
    request_id uuid PRIMARY KEY,
    robot_id text NOT NULL REFERENCES control_robot_slots(robot_id),
    action text NOT NULL CHECK (action IN ('registration','config','exclude','unregister')),
    payload jsonb NOT NULL,
    result jsonb NOT NULL,
    actor text NOT NULL,
    created_at timestamptz NOT NULL DEFAULT now()
);
