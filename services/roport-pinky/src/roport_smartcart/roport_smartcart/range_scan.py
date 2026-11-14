"""Conservative obstacle arc from one Range: never infer a measured bearing."""
import math


def range_arc(value, minimum, maximum, fov):
    """Every bearing is possible; only the nearest echo-free cone is cleared.

    Maximum range means no echo for this input contract. Hardware publishers
    must calibrate their saturation/no-echo convention before using this node.
    """
    if not all(math.isfinite(v) for v in (value, minimum, maximum, fov)):
        return None
    if not (0 < minimum < maximum and minimum <= value <= maximum
            and 0 < fov < math.pi):
        return None
    # <= 1 cm spacing at maximum range closes gaps in the 1 cm local costmap.
    count = max(3, math.ceil(maximum * fov / .01) + 1)
    if count > 1024:
        return None
    return [math.inf if value == maximum else value] * count


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, Range

    rclpy.init()
    node = rclpy.create_node('range_scan')
    publisher = node.create_publisher(LaserScan, 'smartcart/sonar_scan', 10)

    def observe(message):
        rays = range_arc(message.range, message.min_range, message.max_range,
                         message.field_of_view)
        age = node.get_clock().now().nanoseconds / 1e9 - (
            message.header.stamp.sec + message.header.stamp.nanosec / 1e9)
        if rays is None or not message.header.frame_id or not -.1 <= age <= .3:
            return
        scan = LaserScan()
        scan.header = message.header
        scan.angle_min = -message.field_of_view / 2
        scan.angle_max = message.field_of_view / 2
        scan.angle_increment = message.field_of_view / (len(rays) - 1)
        scan.range_min, scan.range_max = message.min_range, message.max_range
        scan.ranges = rays
        publisher.publish(scan)

    node.create_subscription(Range, 'us_sensor/range', observe, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
