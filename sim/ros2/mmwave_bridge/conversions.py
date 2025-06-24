from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

def gz_pointcloud_to_ros2(gz_msg, frame_id="mmwave_sensor_link"):
    """
    Converts a Gazebo PointCloudPacked message to a ROS2 PointCloud2 message.

    Args:
        gz_msg: The input gz.msgs.pointcloud_packed_pb2.PointCloudPacked message.
        frame_id: The TF frame ID for the ROS2 message header.

    Returns:
        A sensor_msgs.msg.PointCloud2 message.
    """
    ros2_msg = PointCloud2()

    # 1. Header
    ros2_msg.header = Header()
    if gz_msg.header and gz_msg.header.stamp:
        ros2_msg.header.stamp.sec = gz_msg.header.stamp.sec
        ros2_msg.header.stamp.nanosec = gz_msg.header.stamp.nsec
    ros2_msg.header.frame_id = frame_id

    # 2. PointFields
    # The protobuf issues can corrupt the gz_msg.field data. We will define the
    # fields manually to ensure correctness. This assumes the sensor produces
    # standard "x, y, z, intensity" points.
    # The sensor plugin produces a 24-byte point structure.
    # Based on the plugin's capabilities (including Doppler) and standard radar formats,
    # the structure is: x, y, z, intensity, range, and doppler_velocity.
    ros2_msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='range', offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name='doppler_velocity', offset=20, datatype=PointField.FLOAT32, count=1),
    ]

    # 3. Cloud properties
    ros2_msg.is_bigendian = gz_msg.is_bigendian
    ros2_msg.point_step = 24  # 6 fields x 4 bytes/field (float32)
    ros2_msg.row_step = gz_msg.row_step
    ros2_msg.height = gz_msg.height
    ros2_msg.width = gz_msg.width

    # is_dense is not in PointCloudPacked, so we assume it's dense
    ros2_msg.is_dense = True

    # 4. Data
    ros2_msg.data = gz_msg.data

    return ros2_msg
