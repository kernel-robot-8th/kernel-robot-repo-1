"""Gazebo-only ray fan to Range; this does not simulate ultrasound reflections."""
import math


def nearest_range(ranges, minimum, maximum):
    """Use finite max for no echo: Jazzy Collision Monitor rejects +infinity.

    Invalid frames are dropped, so source_timeout stops the monitor. A -infinity
    ray represents a too-close return and is conservatively mapped to minimum.
    """
    if not ranges or not (math.isfinite(minimum) and math.isfinite(maximum)
                          and 0 < minimum < maximum):
        return None
    result = maximum
    for value in ranges:
        if math.isnan(value) or (math.isfinite(value) and not minimum <= value <= maximum):
            return None
        result = min(result, minimum if value == -math.inf else value)
    return result


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, Range

    rclpy.init()
    node = rclpy.create_node('sim_sonar')
    publisher = node.create_publisher(Range, 'us_sensor/range', 10)

    def observe(scan):
        value = nearest_range(scan.ranges, scan.range_min, scan.range_max)
        fov = scan.angle_max - scan.angle_min
        age = node.get_clock().now().nanoseconds / 1e9 - (
            scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9)
        if value is None or not 0 < fov < math.pi or not -.1 <= age <= .3:
            return
        message = Range()
        message.header = scan.header
        message.radiation_type = Range.ULTRASOUND
        message.field_of_view = fov
        message.min_range, message.max_range = scan.range_min, scan.range_max
        message.range = value
        publisher.publish(message)

    subscription = node.create_subscription(
        LaserScan, 'sim/sonar_scan', observe, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.try_shutdown()
