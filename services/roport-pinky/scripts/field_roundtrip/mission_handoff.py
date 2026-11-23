"""Per-trip N12 observations, bound to the robot, boot and exact field assets."""
import copy
import math
from pathlib import Path
import re

import yaml


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_of(state):
    pose = state.get('pose')
    if (not state.get('nav2_ready') or not state.get('localization_ready')
            or not state.get('stopped') or not isinstance(pose, dict)
            or any(type(pose.get(k)) not in (int, float) or not math.isfinite(pose[k])
                   for k in ('x', 'y', 'yaw'))):
        raise ValueError('N12 observation requires fresh localized standstill')
    return {k: pose[k] for k in ('x', 'y', 'yaw')}


def capture(context, state, ir, ir_age, ir_ahead, line_hint, outbound_heading):
    if (not re.fullmatch('[0-9a-f]{32}', context.get('trip_id', ''))
            or not context.get('boot_id') or not context.get('robot')
            or any(not context.get(k) for k in ('map_version', 'layout_sha256', 'places_sha256', 'ir_sha256'))):
        raise ValueError('Missing N12 trip identity')
    pose = pose_of(state)
    if (type(ir_ahead) not in (int, float) or not math.isfinite(ir_ahead) or not 0 < ir_ahead <= .1
            or not math.isfinite(ir_age) or not 0 <= ir_age <= .3
            or ir.get('node') is not True or ir.get('lost') is not False
            or ir.get('on_line') != [True, True, True]
            or type(ir.get('error')) not in (int, float) or not math.isfinite(ir['error'])
            or abs(ir['error']) > .25):
        raise ValueError('N12 capture requires a new physical IR bar observation')
    line = {'x': pose['x']+ir_ahead*math.cos(pose['yaw']),
            'y': pose['y']+ir_ahead*math.sin(pose['yaw'])}
    if (math.hypot(line['x']-line_hint['x'], line['y']-line_hint['y']) > .08
            or abs(wrap(pose['yaw']-outbound_heading)) > .1):
        raise ValueError('Observed bar is outside the N12 capture envelope')
    heading = wrap(pose['yaw']+math.pi)
    # The front IR sensor must meet the same bar after reversing direction.
    returned = {'x': line['x']-ir_ahead*math.cos(heading),
                'y': line['y']-ir_ahead*math.sin(heading), 'yaw': heading}
    return {'version': 1, 'context': dict(context),
            'observation': {k: copy.deepcopy(state[k]) for k in ('nav2_ready', 'localization_ready', 'stopped', 'pose')},
            'ir': copy.deepcopy(ir), 'ir_age': ir_age, 'ir_ahead': ir_ahead,
            'return_n12': returned, 'entry_observation': None, 'return_entry': None}


def set_entry(record, state, expected_entry):
    pose = pose_of(state)
    if (math.hypot(pose['x']-expected_entry['x'], pose['y']-expected_entry['y']) > .08
            or abs(wrap(pose['yaw']-expected_entry['yaw'])) > .1):
        raise ValueError('Observed public-space entry differs from the verified entry')
    result = copy.deepcopy(record)
    result['entry_observation'] = {k: copy.deepcopy(state[k]) for k in ('nav2_ready', 'localization_ready', 'stopped', 'pose')}
    result['return_entry'] = {**pose, 'yaw': result['return_n12']['yaw']}
    return result


def validate(record, context, ir_ahead, line_hint, outbound_heading, expected_entry,
             require_entry=True):
    if record.get('version') != 1 or record.get('context') != context or record.get('ir_ahead') != ir_ahead:
        raise ValueError('N12 record belongs to a different trip, boot or field configuration')
    rebuilt = capture(context, record['observation'], record['ir'], record['ir_age'],
                      ir_ahead, line_hint, outbound_heading)
    if record.get('entry_observation') is not None:
        rebuilt = set_entry(rebuilt, record['entry_observation'], expected_entry)
    if (rebuilt['return_n12'] != record.get('return_n12')
            or rebuilt['return_entry'] != record.get('return_entry')):
        raise ValueError('N12 derived goal differs from its recorded physical observation')
    if require_entry and rebuilt['return_entry'] is None:
        raise ValueError('No recorded public-space entry for this trip')
    return rebuilt


def write_places(base_file, record, destination):
    """Write only private runtime goals; never modify the surveyed source YAML."""
    data = yaml.safe_load(Path(base_file).read_text())
    for name in ('return_n12', 'return_entry'):
        data['places'].pop(name, None)
        pose = record[name]
        if pose is not None:
            data['places'][name] = {
                'measured': True, 'pose': [pose[k] for k in ('x', 'y', 'yaw')],
                'measurement_note': 'Physical IR handoff trip '+record['context']['trip_id']}
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    pending = destination.with_suffix('.tmp')
    pending.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    pending.replace(destination)
