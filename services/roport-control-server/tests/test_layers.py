"""Import boundaries per layer, checked with stdlib ast (no extra dependency)."""

import ast
import sys
from pathlib import Path

import pytest

PACKAGE = "roport_control_server"
ROOT = Path(__file__).resolve().parents[1] / "src" / PACKAGE
DB_DRIVERS = {"psycopg", "psycopg2", "asyncpg", "sqlite3", "sqlalchemy", "pymysql"}
STDLIB = set(sys.stdlib_module_names) - DB_DRIVERS
P = f"{PACKAGE}."

# folder -> (third-party top-level names, internal module prefixes). stdlib is always
# allowed except DB drivers. A folder may always import itself. Rules for folders
# that do not exist yet (domain, executor) apply automatically once they appear.
RULES = {
    "": (set(), set()),  # package root __init__ and adapters/__init__
    "contracts": (set(), set()),
    "domain": (set(), set()),
    "executor": (
        set(),
        {P + "domain", P + "contracts", P + "adapters.mqtt", P + "adapters.udp"}
        | {P + "adapters.db", P + "adapters.wss"},
    ),
    "adapters.mqtt": ({"paho"}, set()),
    "adapters.udp": (set(), set()),
    "adapters.db": ({"psycopg"}, {P + "contracts"}),
    "adapters.api": ({"fastapi", "PIL", "yaml"}, {P + "contracts", P + "adapters.db"}),
}


def layer(module: str) -> str:
    parts = module.split(".")[1:]
    if parts[:1] == ["adapters"]:
        return ".".join(parts[:2]) if len(parts) > 1 else ""
    return parts[0] if parts else ""


def violations(module: str, source: str, *, is_package: bool = False) -> list[str]:
    name = layer(module)
    if name not in RULES:
        return [f"{module}: no import rule for layer {name!r}; add one to RULES"]
    third_party, internal = RULES[name]
    own = P + name if name else None
    package = module if is_package else module.rpartition(".")[0]
    targets = []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            targets += [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            base = node.module or ""
            if node.level:
                anchor = package.split(".")[: len(package.split(".")) - node.level + 1]
                base = ".".join(anchor + ([base] if base else []))
            targets += [f"{base}.{alias.name}" for alias in node.names]
    found = []
    for target in targets:
        top = target.split(".")[0]
        if top == PACKAGE:
            allowed = internal | ({own} if own else set())
            ok = any(target == p or target.startswith(p + ".") for p in allowed)
        else:
            ok = top in STDLIB or top in third_party
        if not ok:
            found.append(f"{module} imports {target}")
    return found


def modules():
    for path in sorted(ROOT.rglob("*.py")):
        relative = path.relative_to(ROOT.parent).with_suffix("")
        is_package = relative.name == "__init__"
        parts = relative.parts[:-1] if is_package else relative.parts
        yield ".".join(parts), path, is_package


@pytest.mark.parametrize("module, path, is_package", list(modules()), ids=str)
def test_layer_imports(module, path, is_package):
    assert violations(module, path.read_text(), is_package=is_package) == []


@pytest.mark.parametrize(
    "module, source",
    [
        (P + "contracts.snapshot", "import psycopg"),
        (P + "contracts.snapshot", "import fastapi"),
        (P + "adapters.api.registry", "import psycopg"),
        (P + "adapters.db.registry", "import fastapi"),
        (P + "adapters.mqtt.mqtt", f"from {P}adapters.db import registry"),
        (P + "adapters.udp.udp_frame_sender", "from ..mqtt import messages"),
        (P + "adapters.mqtt.mqtt", "import sqlite3"),
        (P + "adapters.ui.window", f"from {P}adapters.mqtt import mqtt"),
        (P + "adapters.api.app", "import mqtt_demo"),
        (P + "adapters.api.app", "from tests import fakes"),
        (P + "domain.rules", "import paho.mqtt.client"),
        (P + "executor.run", f"from {P}adapters.ui import window"),
        (P + "adapters.newthing.x", "import json"),
        (P + "app", "import json"),
    ],
)
def test_checker_flags_forbidden_imports(module, source):
    assert violations(module, source)


def test_checker_flags_package_relative_import_across_adapters():
    # An adapters/udp/__init__.py importing a sibling adapter must be caught.
    assert violations(P + "adapters.udp", "from .. import mqtt", is_package=True)


def test_checker_allows_expected_imports():
    assert not violations(P + "adapters.api.app", f"from {P}contracts import x")
    assert not violations(P + "adapters.udp.udp_frame_sender", "from .x import y")
    assert not violations(P + "executor.run", f"from {P}domain.state import Step")
    ui_init = P + "adapters.api"
    assert not violations(ui_init, "from .app import app", is_package=True)
