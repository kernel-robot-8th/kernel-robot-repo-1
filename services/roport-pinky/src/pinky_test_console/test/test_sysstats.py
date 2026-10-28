import os

from pinky_test_console.sysstats import (
    memory_summary,
    parse_cpu_temp,
    parse_loadavg,
    parse_meminfo,
    parse_status_kb,
    read_system_stats,
)

MEMINFO_PI5 = """MemTotal:        8245328 kB
MemFree:         5521408 kB
MemAvailable:    6914224 kB
Buffers:           61264 kB
Cached:          1409712 kB
SwapCached:            0 kB
SwapTotal:        204784 kB
SwapFree:         150000 kB
HugePages_Total:       0
"""


def test_parse_meminfo():
    m = parse_meminfo(MEMINFO_PI5)
    assert m['MemTotal'] == 8245328
    assert m['MemAvailable'] == 6914224
    assert m['HugePages_Total'] == 0


def test_memory_summary_used_is_total_minus_available():
    s = memory_summary(parse_meminfo(MEMINFO_PI5))
    assert s['used_kb'] == 8245328 - 6914224
    assert s['available_kb'] == 6914224
    assert s['used_pct'] == round(100 * (8245328 - 6914224) / 8245328, 1)
    assert s['swap_total_kb'] == 204784
    assert s['swap_used_kb'] == 204784 - 150000


def test_memory_summary_missing_fields():
    assert memory_summary(parse_meminfo('MemTotal: 100 kB\n')) is None
    assert memory_summary(parse_meminfo('')) is None


def test_parse_loadavg():
    assert parse_loadavg('0.52 0.38 0.30 2/345 12345\n') == [0.52, 0.38, 0.30]
    assert parse_loadavg('') is None
    assert parse_loadavg('x y z') is None


def test_parse_cpu_temp_millidegree():
    assert parse_cpu_temp('54750\n') == 54.8
    assert parse_cpu_temp('') is None
    assert parse_cpu_temp(None) is None


def test_parse_status_kb():
    text = 'Name:\tpython3\nVmHWM:\t   61234 kB\nVmRSS:\t   58000 kB\n'
    assert parse_status_kb(text, 'VmRSS') == 58000
    assert parse_status_kb(text, 'VmHWM') == 61234
    assert parse_status_kb(text, 'VmSwap') is None


def test_read_system_stats_from_files(tmp_path):
    (tmp_path / 'meminfo').write_text(MEMINFO_PI5)
    (tmp_path / 'loadavg').write_text('1.00 0.50 0.25 1/100 1\n')
    (tmp_path / 'temp').write_text('61000\n')
    (tmp_path / 'status').write_text('VmRSS:\t 40960 kB\nVmHWM:\t 51200 kB\n')
    s = read_system_stats(str(tmp_path / 'meminfo'), str(tmp_path / 'loadavg'),
                          str(tmp_path / 'temp'), str(tmp_path / 'status'))
    assert s['memory']['used_kb'] == 8245328 - 6914224
    assert s['loadavg'] == [1.0, 0.5, 0.25]
    assert s['cpu_temp_c'] == 61.0
    assert s['console_rss_kb'] == 40960
    assert s['console_peak_rss_kb'] == 51200


def test_read_system_stats_missing_files(tmp_path):
    missing = str(tmp_path / 'nope')
    s = read_system_stats(missing, missing, missing, missing)
    assert s == {'memory': None, 'loadavg': None, 'cpu_temp_c': None,
                 'console_rss_kb': None, 'console_peak_rss_kb': None}


def test_read_real_proc():
    """리눅스 /proc 에서 실제로 읽힌다 (온도는 기기에 따라 없을 수 있음)."""
    if not os.path.exists('/proc/meminfo'):
        return
    s = read_system_stats()
    assert s['memory']['total_kb'] > 0
    assert s['console_rss_kb'] > 0
