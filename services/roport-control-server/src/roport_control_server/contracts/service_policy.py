"""Manifest changes are allowed only before a durable dispatch claim."""


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
