"""Offline mission sequencing; transport, motion and durable records live elsewhere."""

from dataclasses import dataclass
from uuid import uuid4


class TransitionError(ValueError):
    """The event does not belong to the current operation."""


@dataclass
class Mission:
    state: str = 'IDLE_EMPTY'
    loaded: bool = False
    destination: str | None = None
    wait_bay: str | None = None
    operation: str | None = None
    stop_result: str | None = None

    @classmethod
    def after_restart(cls, *, loaded: bool):
        """Never resume movement from a saved phase automatically."""
        return cls(state='RECOVERY_REQUIRED', loaded=loaded)

    def _enter(self, state):
        self.state = state
        self.operation = uuid4().hex
        return self.operation

    def _expect(self, state, operation):
        if self.state != state or operation != self.operation or operation is None:
            raise TransitionError('wrong state or stale operation')

    def start_loading(self):
        if self.state != 'IDLE_EMPTY' or self.loaded:
            raise TransitionError('cart must be idle and empty')
        return self._enter('TO_LOAD_LINE')

    def assign_wait_bay(self, place_id, operation):
        """Adapter must verify the control reservation before assigning H1/H2."""
        if self.state not in {'WAIT_LOAD', 'WAIT_UNLOAD'}:
            raise TransitionError('wait bay can only be assigned before a return')
        self._expect(self.state, operation)
        if place_id not in ('wait_p1', 'wait_p2'):
            raise TransitionError('wait bay must be H1 or H2')
        self.wait_bay = place_id

    def dispatch(self, destination):
        if self.state != 'WAIT_DISPATCH' or not self.loaded:
            raise TransitionError('cart must wait with cargo')
        if not isinstance(destination, str) or not destination.strip():
            raise TransitionError('destination must be a configured place ID')
        self.destination = destination
        self.wait_bay = None  # A departed bay needs a fresh control reservation.
        return self._enter('TO_DELIVERY_NAV')

    def complete(self, event, operation, *, place_id=None):
        """Only real adapter results or explicit cargo confirmations advance phases."""
        transitions = {
            'load_route_finished': ('TO_LOAD_LINE', 'WAIT_LOAD'),
            'load_confirmed': ('WAIT_LOAD', 'RETURN_LOADED_LINE'),
            'wait_route_finished': ('RETURN_LOADED_LINE', 'WAIT_DISPATCH'),
            'delivery_reached': ('TO_DELIVERY_NAV', 'WAIT_UNLOAD'),
            'unload_confirmed': ('WAIT_UNLOAD', 'RETURN_NAV'),
            'home_reached': ('RETURN_NAV', 'IDLE_EMPTY'),
        }
        if event not in transitions:
            raise TransitionError('unknown event')
        before, after = transitions[event]
        self._expect(before, operation)
        if event in {'load_confirmed', 'unload_confirmed'} and self.wait_bay is None:
            raise TransitionError('a reserved wait bay is required before returning')
        if event in {'wait_route_finished', 'home_reached'} and (
                self.wait_bay is None or place_id != self.wait_bay):
            raise TransitionError('arrival must match the reserved wait bay')
        if event == 'load_confirmed':
            self.loaded = True
        elif event == 'unload_confirmed':
            self.loaded = False
        elif event == 'home_reached':
            self.destination = None
        return self._enter(after)

    def stop(self, operation, *, failed=False):
        """Invalidate outstanding results; caller must stop motion before acknowledging."""
        if self.state in {'IDLE_EMPTY', 'STOPPING', 'CANCELED', 'FAULT',
                          'RECOVERY_REQUIRED'}:
            raise TransitionError('no active mission to stop')
        self._expect(self.state, operation)
        self.stop_result = 'FAULT' if failed else 'CANCELED'
        return self._enter('STOPPING')

    def stopped(self, operation):
        self._expect('STOPPING', operation)
        return self._enter(self.stop_result)
