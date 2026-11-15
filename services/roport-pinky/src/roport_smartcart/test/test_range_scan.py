import math

import pytest

from roport_smartcart.range_scan import range_arc


def test_single_distance_covers_the_entire_unknown_bearing_arc():
    rays = range_arc(.44, .02, .95, .26)
    assert len(rays) >= 3
    assert set(rays) == {.44}
    assert .95 * .26 / (len(rays) - 1) <= .01


def test_max_range_clears_without_a_false_obstacle_ring():
    assert all(v == math.inf for v in range_arc(.95, .02, .95, .26))


@pytest.mark.parametrize('args', [
    (math.nan, .02, .95, .26), (math.inf, .02, .95, .26),
    (-.1, .02, .95, .26), (1., .02, .95, .26),
    (.44, 0., .95, .26), (.44, .95, .02, .26),
    (.44, .02, .95, 0.), (.44, .02, .95, math.pi),
    (.44, .02, 1000., 2.),
])
def test_invalid_frames_do_not_refresh_the_obstacle_watchdog(args):
    assert range_arc(*args) is None
