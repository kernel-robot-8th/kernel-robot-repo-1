"""Manifest changes are allowed only before a durable dispatch claim."""

import math
from datetime import datetime, timedelta


def dispatch_at(request, settings):
    # shortcut: fixed lead time; replace with measured preparation/travel estimates.
    seconds = settings.get("dispatch_lead_s", 300)
    if type(seconds) not in (int, float) or not math.isfinite(seconds) or seconds < 0:
        raise ValueError("dispatch_lead_s must be finite and nonnegative")
    return datetime.fromisoformat(request["pickup_at"]) - timedelta(seconds=seconds)


def manifest_change_reason(previous, requested, locked):
    """Only a newer package list may change before dispatch is committed."""
    if any(
        previous[k] != requested[k]
        for k in previous.keys() - {"packages", "order_revision"}
    ):
        return "reservation_fields_changed"
    if requested["order_revision"] <= previous["order_revision"]:
        return "order_revision_not_newer"
    if locked:
        return "dispatch_already_locked"
    return None
