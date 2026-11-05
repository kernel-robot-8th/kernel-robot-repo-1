"""Loopback-only fake Pinky for the complete trial; never imports ROS/drivers."""

import argparse
import json
from copy import deepcopy
from datetime import UTC, datetime
from itertools import pairwise

from roport_control_server.adapters.mqtt.pinky_trial import PinkyTrialClient
from roport_control_server.adapters.mqtt.trial_wire import mqtt_config


class FakePinky:
    def __init__(self, profile, role):
        if role not in profile["robots"]:
            raise ValueError("role is not configured in this trial")
        first_route = (
            "to_delivery"
            if profile.get("scenario") == "single_pinky"
            else "to_loading"
            if role == "logistics"
            else "cart_to_delivery"
        )
        route = profile["routes"][first_route]
        self.state = {
            "node_id": route["start_node"],
            "mode": "line",
            "stopped": True,
            "line_enabled": False,
            "nav2_ready": False,
            "localization_ready": False,
            "drive_owner": "none",
            "graph_version": profile["graph_version"],
            "map_id": profile["map_id"],
            "map_version": profile["map_version"],
            "heading_rad": route["start_heading_rad"],
            "fault": None,
        }
        self.pending = None
        self.execution_count = 0
        self.headings = {
            r["start_node"]: r["start_heading_rad"] for r in profile["routes"].values()
        }
        self.nodes = profile["nodes"]

    def observe(self):
        return {**deepcopy(self.state), "observed_at": datetime.now(UTC).isoformat()}

    def validate_action(self, command):
        if command["type"] == "route.requested":
            path = command["payload"]["route"]
            if any(b not in self.nodes.get(a, []) for a, b in pairwise(path)):
                raise ValueError("invalid_graph_edge")

    def dispatch(self, command):
        self.pending = deepcopy(command)
        self.execution_count += 1

    def stop(self, reason):
        self.state.update(stopped=True, line_enabled=False, drive_owner="none")

    def finish(self, client):
        cmd = self.pending
        if cmd is None:
            return
        self.pending = None
        p = cmd["payload"]
        if cmd["type"] == "route.requested":
            route = p["route"]
            for i, node in enumerate(route):
                self.state.update(
                    heading_rad=self.headings.get(node, self.state["heading_rad"]),
                    node_id=node,
                    stopped=i == len(route) - 1,
                    line_enabled=i != len(route) - 1,
                    drive_owner="line",
                )
                client.report(
                    cmd,
                    "completed" if i == len(route) - 1 else "progress",
                    seq=i,
                    total=len(route),
                    prev_node=route[i - 1] if i else None,
                    next_node=route[i + 1] if i + 1 < len(route) else None,
                )
        elif cmd["type"] == "mode_switch.requested":
            self.state.update(
                mode="nav2",
                stopped=True,
                line_enabled=False,
                nav2_ready=True,
                localization_ready=True,
                drive_owner="nav2",
            )
            client.report(cmd, "completed")
        else:
            client.report(
                cmd,
                "completed",
                goal_id=p["goal_id"],
                goal_version=p["goal_version"],
                frame_id="map",
                pose=p["pose"],
                nav2_result="SUCCEEDED",
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--role", choices=("logistics", "smartcart"), required=True)
    parser.add_argument("--journal", required=True)
    args = parser.parse_args()
    with open(args.config) as file:
        config = json.load(file)
    if args.role not in config["robots"]:
        parser.error("role is not configured in this trial")
    robot = config["robots"][args.role]
    connection = mqtt_config(f"{config['site_id']}-{robot['robot_id']}")
    if config["mode"] != "simulation" or connection.host not in {
        "127.0.0.1",
        "localhost",
        "::1",
    }:
        parser.error("fake robot requires simulation profile and loopback broker")
    fake = FakePinky(config, args.role)
    client = PinkyTrialClient(
        connection,
        site=config["site_id"],
        control=config["control_id"],
        robot=robot["robot_id"],
        binding_id=robot["binding_id"],
        journal_path=args.journal,
        observe=fake.observe,
        validate_action=fake.validate_action,
        dispatch=fake.dispatch,
        safe_stop=fake.stop,
    )
    try:
        client.start()
        print(
            f"READY {robot['robot_id']} boot={client.boot} SIMULATION ONLY", flush=True
        )
        while True:
            client.poll()
            fake.finish(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()


if __name__ == "__main__":
    main()
