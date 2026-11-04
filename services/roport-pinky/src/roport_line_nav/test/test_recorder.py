import csv
import datetime
import os

from sim import Sim

from roport_line_nav.executor import RouteExecutor
from roport_line_nav.graph import Graph
from roport_line_nav.recorder import FILE_RE, RouteRecorder

LAYOUT = os.path.join(os.path.dirname(__file__), '..', '..', 'roport_gz_world', 'layout',
                      'roport_airport.yaml')


def test_records_one_route_per_file(tmp_path):
    graph = Graph.load(LAYOUT)
    sim = Sim(graph)
    sim.place('N14', 'N', 0.0)
    ex = RouteExecutor(graph)
    rec = RouteRecorder(str(tmp_path))
    ex.start(['N14', 'N16'])
    path = rec.open(ex.route, sim.t)
    assert FILE_RE.match(os.path.basename(path)) and path.endswith('_N14-N16.csv')
    while ex.active and sim.t < 120:
        reading = sim.tick(ex)
        rec.write(sim.t, ex.as_dict(), reading, sim.cmd, sim.pose())
    rec.close()
    rows = list(csv.DictReader(open(path)))
    assert len(rows) > 100 and rows[-1]['state'] == 'arrived'
    assert rows[0]['events'].startswith('start:N14 → N16')
    assert any('node:N16' in r['events'] for r in rows)
    assert rec.files()[0]['name'] == os.path.basename(path)


def test_keeps_only_latest(tmp_path):
    rec = RouteRecorder(str(tmp_path), keep=3)
    base = datetime.datetime(2026, 11, 4, 9, 0, 0)
    for i in range(5):
        rec.open(['N14', 'N16'], 0.0, wall=base + datetime.timedelta(minutes=i))
        rec.close()
    names = [f['name'] for f in rec.files()]
    assert names == ['route_20261104_090400_N14-N16.csv', 'route_20261104_090300_N14-N16.csv',
                     'route_20261104_090200_N14-N16.csv']
    (tmp_path / 'other.txt').write_text('x')            # 기록 파일이 아니면 지우지 않음
    rec.prune()
    assert (tmp_path / 'other.txt').exists()


def test_unwritable_dir_disables_only_logging(tmp_path):
    blocker = tmp_path / 'file'
    blocker.write_text('x')
    rec = RouteRecorder(str(blocker / 'sub'))
    assert rec.open(['N14', 'N16'], 0.0) is None and rec.error
    rec.write(0.0, {}, None, None, None)                 # 예외 없이 무시
