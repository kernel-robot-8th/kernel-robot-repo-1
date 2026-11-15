"""라이다 scan → 몸체 앞·뒤 끝에서 가장 가까운 점 거리(FFT1-172 장애물 정지). 순수 Python, ROS 없음.

계약(09_obstacle_yield_contract 3): 점 = range_min ≤ r ≤ range_max 인 유한값만. scan frame 점을 TF(base_link ← scan
frame)로 base_link 로 옮긴 뒤 각도 θ = atan2(y, x) 로 고른다(라이다는 yaw = π 로 붙어 있어 원시 각도를 쓰면 앞뒤가 바뀜).
  front_cm = |θ| ≤ sector_deg 인 점들의 (x − front_edge_cm) 최솟값
  rear_cm  = |θ − π| ≤ sector_deg 인 점들의 (−x − rear_edge_cm) 최솟값
부채꼴에 점이 하나도 없으면 None(가까이 붙은 물체가 range_min 안이라 무효로 나온 경우도 None — 실행부가 판단).
"""

import math


def planar_tf(tx_m, ty_m, yaw):
    """평면 TF (base_link ← scan frame). → (tx, ty, r00, r01, r10, r11), m."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (float(tx_m), float(ty_m), c, -s, s, c)


def tf_from_quaternion(tx_m, ty_m, qx, qy, qz, qw):
    """TF 회전(쿼터니언)의 위 왼쪽 2x2 와 평행 이동. scan 점은 z = 0 이라 3x3 의 첫 두 열만 쓰면 된다."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError('invalid quaternion')
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    return (float(tx_m), float(ty_m), r00, r01, r10, r11)


def edge_distances(ranges, angle_min, angle_increment, range_min, range_max, tf,
                   sector_deg, front_edge_cm, rear_edge_cm):
    """→ (front_cm, rear_cm), 없으면 None. ranges·range_* 는 m(LaserScan 그대로), 결과는 cm."""
    tx, ty, r00, r01, r10, r11 = tf
    lim = math.radians(sector_deg)
    front = rear = None
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and range_min <= r <= range_max):
            continue
        a = angle_min + i * angle_increment
        lx, ly = r * math.cos(a), r * math.sin(a)
        x = tx + r00 * lx + r01 * ly
        y = ty + r10 * lx + r11 * ly
        th = math.atan2(y, x)
        if abs(th) <= lim:
            v = x * 100.0 - front_edge_cm
            if front is None or v < front:
                front = v
        elif abs(math.pi - abs(th)) <= lim:
            v = -x * 100.0 - rear_edge_cm
            if rear is None or v < rear:
                rear = v
    return front, rear
