from datetime import datetime
import pytz

try:
    from jsk_ros1_ros2_compat.rospy_rclpy_compat import stamp_to_sec
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only
    def stamp_to_sec(stamp):
        return stamp.to_sec()


def timestamp_to_influxdb_time(timestamp):
    time = datetime.utcfromtimestamp(stamp_to_sec(timestamp))
    time = pytz.timezone('UTC').localize(time).isoformat('T')
    return time
