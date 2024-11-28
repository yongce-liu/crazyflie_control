import rclpy
import numpy as np

from geometry_msgs.msg import PoseStamped


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