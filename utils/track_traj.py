import rclpy
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from crazyflie_interfaces.msg import LogDataGeneric

from utils.data_template import DataLoader

class TrajTracking(rclpy.node.Node):
    def __init__(self, crazyflie, traj_root_path):
        self.prefix = crazyflie.prefix
        super().__init__(self.prefix[1:])

        self.cf = crazyflie
        # must have for trajectory:
        # - rate
        # - traj
        traj_data = DataLoader(traj_root_path+f"{self.prefix}.npz")
        self.traj = traj_data.position
        self.rate = traj_data.rate
        self.traj_length = len(self.traj)
        self.traj_current_idx = 0
        # self.initPosition = self.cf.initialPosition

        self.position = []
        self.position_buffer = []
        self.velocity = []

        # subscriber
        self.create_subscription(PoseStamped, f"{self.prefix}/pose", self._position_msg_callback, 10)
        # self.create_subscription(LogDataGeneric, f'{self.prefix}/velocity', self._velocity_msg_callback, 10)

        # publisher
        self.traj_publisher = self.create_publisher(Path, f"{self.prefix}/ref_trajectory", 10)

        # self.publish_traj_alltime()

        # go to the initPosition
        self.cf.goTo(np.array(self.traj[0]), 0.0, duration=3.)
        # self.sleep(duration=4.0)

        self.get_logger().info("Initialization completed...")

        self.timer = self.create_timer(1 / self.rate, self._main_loop)

    def _position_msg_callback(self, msg: PoseStamped):
        self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.position_buffer.append(self.position)
        # self.get_logger().info(f'{self.position}')

    def _velocity_msg_callback(self, msg: LogDataGeneric):
        self.velocity = msg.values

    def _attitude_msg_callback(self, msg: LogDataGeneric):
        self.attitude = msg.values

    def publish_traj_alltime(self):
        solution_path = Path()
        solution_path.header.frame_id = 'world'
        solution_path.header.stamp = self.get_clock().now().to_msg()
        for i in range(self.traj_length):
            pose = PoseStamped()
            pose.pose.position.x = self.traj[i][0]
            pose.pose.position.y = self.traj[i][1]
            # pose.pose.position.z = 1.
            pose.pose.position.z = self.traj[i][2]
            solution_path.poses.append(pose)

        self.traj_publisher.publish(solution_path)

    def publish_traj(self, start_index, length):
        solution_path = Path()
        solution_path.header.frame_id = "world"
        solution_path.header.stamp = self.get_clock().now().to_msg()
        for i in range(max(start_index, 0), min(start_index + length, self.traj_length)):
            pose = PoseStamped()
            pose.pose.position.x = self.traj[i][0]
            pose.pose.position.y = self.traj[i][1]
            # pose.pose.position.z = 1.
            pose.pose.position.z = self.traj[i][2]
            solution_path.poses.append(pose)

        self.traj_publisher.publish(solution_path)

    # def sleep(self, duration):
    #     start = self.get_clock().now().nanoseconds
    #     end = start + duration * 1e9
    #     while self.get_clock().now().nanoseconds < end:
    #         pass

    def _main_loop(self):
        # if not self.position or not  self.velocity or not self.attitude:
        #     self.get_logger().warning("Empty state message & Not Connected.")
        #     return
        if not self.position:
            self.get_logger().warning("Empty state message & Not Connected.")
            return

        if self.traj_current_idx == self.traj_length:
            self.cf.goTo(np.array(self.traj[-1]), 0.0, duration=1.0)
            self.get_logger().info("Trajectory Completed...")
            # self.get_logger().info('Timer cycle completed, shutting down...')
            self.timer.cancel()
            self.destroy_node()
            return

        self.publish_traj(start_index=self.traj_current_idx - 5, length=10)

        temp_position = np.array(self.traj[self.traj_current_idx])
        # self.cf.cmdPosition(temp_position, yaw=self.traj.attitude[i][-1])
        # self.cf.goTo(temp_position, self.traj.attitude[i][-1], duration=1 / self.rate)
        self.cf.goTo(temp_position, 0.0, duration=1 / self.rate)
        self.traj_current_idx = self.traj_current_idx + 1
