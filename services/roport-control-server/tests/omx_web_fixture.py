"""PR45 mock Host on the browser fixture's isolated broker; no hardware imports."""

import json
import sys
from pathlib import Path

OMX = Path(__file__).resolve().parents[2] / "roport-omx"
sys.path[:0] = [str(OMX / "src"), str(OMX / "scripts")]
from cooperative_host import serve
from cooperative_mock import MockExecutor
from roport_omx.cooperative import CooperativeHost
from roport_omx.mqtt_link.journal import Journal
from roport_omx.mqtt_link.messages import topic

from roport_control_server.adapters.mqtt.mqtt import MqttTransport
from roport_control_server.adapters.mqtt.trial_wire import mqtt_config

profile = json.loads(Path(sys.argv[1]).read_text())
with_path = Journal(sys.argv[2])
transport = MqttTransport(
    mqtt_config(profile["control_id"] + "-omx"),
    [
        topic(profile["site_id"], "workcell", "shelf-01", channel)
        for channel in ("cmd", "ack")
    ],
)
try:
    host = CooperativeHost(
        with_path,
        MockExecutor(0.3),
        site=profile["site_id"],
        control=profile["control_id"],
        binding=profile["omx"]["binding_id"],
    )
    transport.start()
    serve(host, transport, profile["site_id"])
except KeyboardInterrupt:
    pass
finally:
    transport.stop()
    with_path.close()
