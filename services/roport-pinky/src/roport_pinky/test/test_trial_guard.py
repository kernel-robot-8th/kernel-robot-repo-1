from roport_pinky.trial_guard import LeaseGate


def lease(g, seq, owner='line', session='runner'):
    return {'boot': g.boot, 'seq': seq, 'owner': owner, 'session': session, 'sent_at': 10.}


def test_fail_closed_source_handoff_and_owner_expiry():
    g = LeaseGate()
    assert g.relay('line', 0, .03, 0) is None
    g.lease(lease(g, 1), 0, 10)
    assert g.relay('line', .1, .03, 0) == (.03, 0)
    assert g.relay('nav2', .2, .1, 0) is None
    g.lease(lease(g, 2, 'nav2'), .3, 10)
    assert g.relay('line', .4, .03, 0) is None
    assert g.relay('nav2', .4, .03, 0) == (.03, 0)
    # A healthy velocity publisher cannot renew the MQTT owner's lease.
    for i in range(5, 13):
        assert g.relay('nav2', i / 10, .03, 0) == (.03, 0)
    assert g.tick(1.31) and g.blocked
    g.lease(lease(g, 3, 'nav2'), 1.32, 10)
    assert g.relay('nav2', 1.33, .03, 0) is None


def test_duplicate_stale_wrong_boot_and_restarted_owner_do_not_renew():
    g = LeaseGate()
    g.lease(lease(g, 1), 0, 10)
    for body in (lease(g, 1), {**lease(g, 2), 'sent_at': 8},
                 {**lease(g, 2), 'boot': 'old'}):
        g.lease(body, .9, 10)
    assert g.tick(1.01) and g.blocked
    g = LeaseGate()
    g.lease(lease(g, 1), 0, 10)
    g.lease(lease(g, 2, session='other-runner'), .1, 10)
    assert g.blocked and g.owner == 'none'


def test_no_late_lease_resurrection_between_timer_ticks():
    g = LeaseGate()
    g.lease(lease(g, 1), 0, 10)
    g.lease(lease(g, 2), 1.01, 10)
    assert g.blocked
