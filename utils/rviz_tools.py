import rclpy
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField


class PonitCloudPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__("point_cloud_publisher")

        # Initialize point cloud data
        self.points = self.generate_point_cloud()

        # Create PointCloud2 publisher
        self.publisher = self.create_publisher(PointCloud2, "/contour", 10)

        # Publish point cloud data
        self.publish_point_cloud()

    def generate_point_cloud(self):
        # Generate sample point cloud data
        # [[0, 1], [0, 1]]
        dat = np.loadtxt(
            DATA_PATH+"dist_point.txt", delimiter=","
        )
        # 坐标变换[0, 1] -> [x_min, x_max]

        x = dat[:, 0]  # Example x data
        y = dat[:, 1]  # Example y data
        z = dat[:, 2]*2.5  # Example z data
        points = np.hstack((x.reshape(-1, 1), y.reshape(-1, 1), z.reshape(-1, 1)))
        return points

    def publish_point_cloud(self):
        # Create PointCloud2 message
        pointcloud_msg = PointCloud2()

        # Fill PointCloud2 message fields
        pointcloud_msg.header.frame_id = "world"  # Assuming coordinate frame is "map"
        pointcloud_msg.height = 1
        pointcloud_msg.width = len(self.points)
        pointcloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),  # Intensity field for color
        ]
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 16  # Size of each point in bytes
        pointcloud_msg.row_step = pointcloud_msg.point_step * len(self.points)
        pointcloud_msg.is_dense = False

        # Calculate intensity values (color based on z)
        intensities = (self.points[:, 2] - np.min(self.points[:, 2])) / (
            np.max(self.points[:, 2]) - np.min(self.points[:, 2])
        )

        # Pack point cloud data into bytes
        point_data = np.column_stack((self.points, intensities)).astype(np.float32)
        pointcloud_msg.data = point_data.tobytes()

        # Publish PointCloud2 message
        self.publisher.publish(pointcloud_msg)
        self.get_logger().info("Point cloud published")