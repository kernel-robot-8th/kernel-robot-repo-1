"""IR 기록 로직 테스트 (ROS·센서 없이 가짜 시계로)."""

import csv

import pytest

from pinky_test_console.ir_record import (
    channel_stats,
    IrRecorder,
    IrRecordError,
    suggest_thresholds,
    validate_filename,
)


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def rec(tmp_path, clock):
    return IrRecorder(str(tmp_path / 'ir'), clock=clock)


def feed(rec, clock, values, seconds, hz=20):
    for _ in range(int(seconds * hz)):
        clock.t += 1.0 / hz
        rec.add_sample(values)


def test_snapshot_without_data(rec):
    s = rec.snapshot()
    assert s['latest'] is None and s['hz'] is None and s['recording'] is None


def test_latest_rate_and_history_window(rec, clock):
    feed(rec, clock, [100, 2000, 300], seconds=15)
    s = rec.snapshot()
    assert s['latest'] == [100, 2000, 300]
    assert s['hz'] == pytest.approx(20, abs=0.5)
    assert s['history'][0][0] >= -10.0          # 최근 10 s 만 유지
    assert len(s['history']) <= 201


def test_short_message_ignored(rec):
    rec.add_sample([1, 2])
    assert rec.snapshot()['latest'] is None


def test_record_writes_csv_and_stats(rec, clock, tmp_path):
    rec.start('carpet', 2)
    feed(rec, clock, [1000, 1100, 1200], seconds=3)
    result = rec.snapshot()['last_result']
    assert result['success'] and result['stats']['n'] == 40
    assert result['stats']['channels'][1] == {'mean': 1100, 'std': 0, 'min': 1100, 'max': 1100}
    with open(tmp_path / 'ir' / result['file']) as f:
        rows = list(csv.reader(f))
    assert rows[0] == ['t_sec', 'ir0', 'ir1', 'ir2'] and len(rows) == 41
    assert result['file'].endswith('_carpet.csv')


def test_record_finishes_without_samples(rec, clock):
    rec.start('tape_center', 1)
    clock.t += 2
    result = rec.snapshot()['last_result']
    assert not result['success'] and 'main_node' in result['msg']
    assert rec.snapshot()['recording'] is None


def test_record_rejects_bad_input_and_double_start(rec):
    for label in ('', 'a b', '../x', 'x' * 41, 'ok;rm'):
        with pytest.raises(IrRecordError):
            rec.start(label, 5)
    for dur in (0, 61, True, 'x', float('nan')):
        with pytest.raises(IrRecordError):
            rec.start('ok', dur)
    rec.start('ok', 5)
    with pytest.raises(IrRecordError, match='already'):
        rec.start('ok2', 5)
    assert rec.cancel() is True and rec.cancel() is False


def test_list_records_and_suggestion(rec, clock):
    rec.start('carpet', 1)
    feed(rec, clock, [3000, 3100, 2900], seconds=1.5)
    clock.t += 1          # 다른 파일 이름(초 단위)이 되도록 실제 시간과 무관하게 진행
    rec.start('tape_center', 1)
    feed(rec, clock, [500, 600, 400], seconds=1.5)
    records = rec.list_records()
    assert {r['label'] for r in records} <= {'carpet', 'tape_center'}
    sg = suggest_thresholds([
        {'label': 'carpet', 'stats': channel_stats([(0, 3000, 3100, 2900), (0, 3010, 3110, 2910)])},
        {'label': 'tape_center', 'stats': channel_stats([(0, 500, 600, 400), (0, 510, 610, 410)])},
    ])
    assert sg['channels'][0]['threshold'] == pytest.approx(1755)
    assert sg['channels'][0]['tape_is_higher'] is False
    assert sg['channels'][0]['separation'] > 3


def test_suggestion_needs_both_groups():
    only = [{'label': 'carpet', 'stats': channel_stats([(0, 1, 2, 3)])}]
    assert suggest_thresholds(only) is None


@pytest.mark.parametrize('name', ['../etc/passwd', '20261028_120000_a.txt', 'x.csv',
                                  '20261028_120000_a/b.csv', '20261028_120000_.csv'])
def test_record_filename_validation(name):
    with pytest.raises(IrRecordError):
        validate_filename(name)


def test_record_path_inside_dir(rec, tmp_path):
    p = rec.record_path('20261028_120000_carpet.csv')
    assert p == str(tmp_path / 'ir' / '20261028_120000_carpet.csv')
