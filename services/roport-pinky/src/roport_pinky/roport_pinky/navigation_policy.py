"""Preserve hardware tracking defaults; short-range tuning is simulation-only."""

RPP_TRACKING = {
    'lookahead_dist': 0.30,
    'min_lookahead_dist': 0.15,
    'max_lookahead_dist': 0.45,
    'lookahead_time': 1.0,
    'curvature_lookahead_dist': 0.30,
    'use_collision_detection': True,
}

SIM_RPP_TRACKING = {
    **RPP_TRACKING,
    'lookahead_dist': 0.15,
    'min_lookahead_dist': 0.08,
    'max_lookahead_dist': 0.20,
    'curvature_lookahead_dist': 0.15,
}
