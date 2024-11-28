import numpy as np
import rclpy
import rclpy.node
from crazyflie_interfaces.msg import LogDataGeneric  # , AttitudeSetpoint
import pathlib
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy import executors
import argparse
from .SelfModule import StateCtrl  # , StateCtrlSim
import pickle as pkl
from crazyflie_py import *
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


import rclpy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import pathlib


import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt

DATA_PATH = str(pathlib.Path(__file__).parent / f"data") + "/"


class CrazyfliePoseSubscriber(rclpy.node.Node):
    def __init__(self, drone_name_list, stop_time):
        super().__init__("drone_pose_subscriber")

        self.get_logger().info("Start sub pose...")

        self.dron_name_list = drone_name_list
        self.pose_dict = {name: [] for name in drone_name_list}
        self.stop_time = stop_time
        self.start_time = self.get_clock().now()

        for name in drone_name_list:
            self.create_subscription(
                PoseStamped, "/" + name + "/pose", self.generate_callback(name), 10
            )
        self.timer = self.create_timer(1 / 50, self._main_loop)

    def generate_callback(self, name):
        def callback(msg: PoseStamped):
            position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.pose_dict[name].append(position)

        return callback

    def save_poses_to_file(self, name):
        file_path = DATA_PATH + name + "/local_pose.txt"

        np.savetxt(
            fname=file_path,
            X=np.array(self.pose_dict[name]),
            fmt="%.10e",
            delimiter=",",
        )
        self.get_logger().info(f"Saved {name}' poses.")
        # self.destroy_node()

    def _main_loop(self):
        if (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9 > self.stop_time:
            for name in self.dron_name_list:
                self.save_poses_to_file(name=name)
            self.timer.cancel()
            self.destroy_node()


class ContourPublisher(rclpy.node.Node):
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


class CrazyflieESMC(rclpy.node.Node):
    def __init__(self, node_name, Crazyflie, rate):
        super().__init__(node_name)

        self.cf = Crazyflie
        prefix = self.cf.prefix
        self.rate = rate
        self.count_num = 0
        # self.initPosition = self.cf.initialPosition

        self.position = []
        self.velocity = []
        self.attitude = []

        traj_path = DATA_PATH + prefix[1:] + "/traj.pkl"
        self.traj = self.load_traj(traj_path)
        self.traj_points_len = len(self.traj.position)

        # go to the initPosition
        # self.cf.goTo(np.array(self.traj.position[0]), 0.0, duration=3.0)
        # self.sleep(duration=4.0)

        self.position_list = []

        self.get_logger().info("Initialization completed...")

        self.create_subscription(
            PoseStamped, f"{prefix}/pose", self._position_msg_callback, 10
        )

        # self.create_subscription(
        #     LogDataGeneric,
        #     f'{prefix}/velocity',
        #     self._velocity_msg_callback,
        #     10)

        # self.create_subscription(
        #     LogDataGeneric,
        #     f'{prefix}/attitude',
        #     self._attitude_msg_callback,
        #     10)

        # self.attitude_setpoint_pub = self.create_publisher(
        #     AttitudeSetpoint,
        #     f'{prefix}/cmd_attitude_setpoint',
        #     10)

        self.traj_pub = self.create_publisher(Path, f"{prefix}/esmc_path", 10)

        # self.publish_traj_alltime()

        self.timer = self.create_timer(1 / rate, self._main_loop)

    def _position_msg_callback(self, msg: PoseStamped):
        self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # self.position_list.append(self.position)
        # self.get_logger().info(f'{self.position}')

    def _velocity_msg_callback(self, msg: LogDataGeneric):
        self.velocity = msg.values

    def _attitude_msg_callback(self, msg: LogDataGeneric):
        self.attitude = msg.values

    def load_traj(self, path):
        with open(path, "rb") as f:
            traj_dict = pkl.load(f)
        traj = StateCtrl(state_control_dict=traj_dict)
        # for i in range(len(traj.position)):
        #     traj.position[i][2] = traj.position[i][2]-0.5
        # traj = StateCtrlSim(state_control_dict=traj_dict)

        return traj

    def publish_traj_alltime(self):
        solution_path = Path()
        solution_path.header.frame_id = 'world'
        solution_path.header.stamp = self.get_clock().now().to_msg()
        position = self.traj.position
        for i in range(len(position)):
            pose = PoseStamped()
            pose.pose.position.x = position[i][0]
            pose.pose.position.y = position[i][1]
            # pose.pose.position.z = 1.
            pose.pose.position.z = position[i][2]
            solution_path.poses.append(pose)

        self.traj_pub.publish(solution_path)

    def publish_traj(self, start_index, length):
        solution_path = Path()
        solution_path.header.frame_id = "world"
        solution_path.header.stamp = self.get_clock().now().to_msg()
        position = self.traj.position
        for i in range(max(start_index, 0), min(start_index + length, len(position))):
            pose = PoseStamped()
            pose.pose.position.x = position[i][0]
            pose.pose.position.y = position[i][1]
            # pose.pose.position.z = 1.
            pose.pose.position.z = position[i][2]
            solution_path.poses.append(pose)

        self.traj_pub.publish(solution_path)

    def sleep(self, duration):
        start = self.get_clock().now().nanoseconds
        end = start + duration * 1e9
        # while True:
        #     if self.get_clock().now().nanoseconds > end:
        #         break
        while self.get_clock().now().nanoseconds < end:
            pass
            # rclpy.spin_once(self.node, timeout_sec=0)

    def _main_loop(self):
        # if not self.position or not  self.velocity or not self.attitude:
        #     self.get_logger().warning("Empty state message & Not Connected.")
        #     return

        if not self.position:
            self.get_logger().warning("Empty state message & Not Connected.")
            return

        # if self.count_num == int(self.traj.duration*self.rate):
        if self.count_num == int(self.traj_points_len):
            self.cf.goTo(np.array(self.traj.position[-1]), 0.0, duration=1.0)
            self.sleep(duration=2.0)
            # self.timeHelper.sleep(2.0)
            self.cf.land(targetHeight=0.06, duration=3.0)
            self.sleep(duration=5.0)
            # self.timeHelper.sleep(5.0)
            self.get_logger().info("Trajectory Completed...")
            # self.get_logger().info('Timer cycle completed, shutting down...')
            self.timer.cancel()  # 取消定时器

            self.destroy_node()
            return
        
        i = self.count_num

        self.publish_traj(start_index=i - 5, length=10)

        temp_position = np.array(self.traj.position[i])
        # self.cf.cmdPosition(temp_position, yaw=self.traj.attitude[i][-1])
        # self.cf.goTo(temp_position, self.traj.attitude[i][-1], duration=1 / self.rate)
        if self.count_num==0:
            self.cf.goTo(temp_position, 0.0, duration=3.0)
            self.sleep(duration=5.0)
        else:
            self.cf.goTo(temp_position, 0.0, duration=1 / self.rate)
        self.count_num = self.count_num + 1


def main():
    swarm = Crazyswarm()

    # collision avoidance
    # swarm.allcfs.setParam("colAv.enable", 1)
    # swarm.allcfs.setParam("colAv.ellipsoidX", 0.15)
    # swarm.allcfs.setParam("colAv.ellipsoidY", 0.15)
    # swarm.allcfs.setParam("colAv.ellipsoidZ", 0.2)

    swarm.allcfs.takeoff(targetHeight=1.0, duration=3.0)
    swarm.timeHelper.sleep(5.0)

    nodes_1 = [
        ContourPublisher(),
        CrazyfliePoseSubscriber(
            drone_name_list=[cf.prefix[1:] for cf in swarm.allcfs.crazyflies],
            stop_time=60+5*2+2+3,
            # stop_time=20+5*2+2+3,
        ),
    ]
    nodes_2 = [
        CrazyflieESMC(node_name=cf.prefix[1:], Crazyflie=cf, rate=2.0)
        # CrazyflieESMC(node_name=cf.prefix[1:], Crazyflie=cf, rate=5.0)
        for cf in swarm.allcfs.crazyflies
    ]
    nodes = nodes_1 + nodes_2
    executor = executors.MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    try:
        while rclpy.ok():
            node.get_logger().info(
                "Beginning multiagent executor, shut down with CTRL-C"
            )
            executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")

    # for node in nodes:
    #     node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
