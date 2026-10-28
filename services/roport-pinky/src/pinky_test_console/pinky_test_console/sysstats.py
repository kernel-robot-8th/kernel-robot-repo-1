"""/proc, /sys 에서 메모리·부하·온도 읽기 (ROS 의존성 없음, 단위 테스트 대상).

모든 메모리 값 단위는 kB (/proc/meminfo 와 동일, 1 kB = 1024 bytes).
"""

MEMINFO = '/proc/meminfo'
LOADAVG = '/proc/loadavg'
CPU_TEMP = '/sys/class/thermal/thermal_zone0/temp'
SELF_STATUS = '/proc/self/status'


def _read(path):
    try:
        with open(path, 'r') as f:
            return f.read()
    except OSError:
        return None


def parse_meminfo(text):
    """'MemTotal:  8000 kB' 형식을 {'MemTotal': 8000, ...} 로."""
    out = {}
    for line in (text or '').splitlines():
        key, sep, rest = line.partition(':')
        if not sep:
            continue
        parts = rest.split()
        if parts:
            try:
                out[key.strip()] = int(parts[0])
            except ValueError:
                pass
    return out


def memory_summary(meminfo):
    """사용량 = MemTotal - MemAvailable (free 명령의 used 와 같은 기준)."""
    total = meminfo.get('MemTotal')
    avail = meminfo.get('MemAvailable')
    if total is None or avail is None:
        return None
    used = total - avail
    swap_total = meminfo.get('SwapTotal', 0)
    swap_free = meminfo.get('SwapFree', 0)
    return {
        'total_kb': total,
        'available_kb': avail,
        'used_kb': used,
        'used_pct': round(100.0 * used / total, 1) if total else None,
        'swap_total_kb': swap_total,
        'swap_used_kb': swap_total - swap_free,
    }


def parse_loadavg(text):
    parts = (text or '').split()
    if len(parts) < 3:
        return None
    try:
        return [float(parts[0]), float(parts[1]), float(parts[2])]
    except ValueError:
        return None


def parse_cpu_temp(text):
    """thermal_zone temp 는 밀리도(m°C). °C 로 변환."""
    try:
        return round(int((text or '').strip()) / 1000.0, 1)
    except ValueError:
        return None


def parse_status_kb(text, key):
    """/proc/<pid>/status 에서 VmRSS, VmHWM 같은 값을 kB 로."""
    for line in (text or '').splitlines():
        if line.startswith(key + ':'):
            parts = line.split()
            if len(parts) >= 2:
                try:
                    return int(parts[1])
                except ValueError:
                    return None
    return None


def read_system_stats(meminfo_path=MEMINFO, loadavg_path=LOADAVG,
                      temp_path=CPU_TEMP, self_status_path=SELF_STATUS):
    status = _read(self_status_path)
    return {
        'memory': memory_summary(parse_meminfo(_read(meminfo_path))),
        'loadavg': parse_loadavg(_read(loadavg_path)),
        'cpu_temp_c': parse_cpu_temp(_read(temp_path)),
        'console_rss_kb': parse_status_kb(status, 'VmRSS'),
        'console_peak_rss_kb': parse_status_kb(status, 'VmHWM'),
    }
