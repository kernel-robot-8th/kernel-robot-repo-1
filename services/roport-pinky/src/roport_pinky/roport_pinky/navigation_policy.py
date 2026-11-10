"""Default short-range tracking for the trial and standalone Nav2 entry points."""

RPP_TRACKING = {
    'lookahead_dist': 0.15,
    'min_lookahead_dist': 0.08,
    'max_lookahead_dist': 0.20,
    'lookahead_time': 1.0,
    'curvature_lookahead_dist': 0.15,
    'use_collision_detection': True,
}
