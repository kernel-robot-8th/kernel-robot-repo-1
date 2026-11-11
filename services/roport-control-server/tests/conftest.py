import sys
from pathlib import Path

pytest_plugins = ["mqtt_helpers", "registry_helpers"]

# The shared fake device (MockExecutor, make_command) lives in scripts/mqtt_demo.py,
# outside the wheel; tests import it from there instead of keeping a second copy.
SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
