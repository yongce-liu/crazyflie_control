import rclpy
import rclpy.node
from rclpy import executors
import pathlib

from crazyflie_py import *

from utils.track_traj import TrajTracking
from utils.rviz_tools import PonitCloudPublisher

def main():
    swarm = Crazyswarm()

    # collision avoidance
    # swarm.allcfs.setParam("colAv.enable", 1)
    # swarm.allcfs.setParam("colAv.ellipsoidX", 0.15)
    # swarm.allcfs.setParam("colAv.ellipsoidY", 0.15)
    # swarm.allcfs.setParam("colAv.ellipsoidZ", 0.2)

    swarm.allcfs.takeoff(targetHeight=1.0, duration=3.0)
    # swarm.allcfs.takeoff(targetHeight=2.19238105, duration=3.0)
    swarm.timeHelper.sleep(5.0)

    data_path = str(pathlib.Path(__file__).parent / f"data")
    # nodes_1 = [PonitCloudPublisher(points_path=data_path+'/bunny.npy')]
    nodes_1 = []
    nodes_2 = [TrajTracking(crazyflie=cf, traj_root_path=data_path) for cf in swarm.allcfs.crazyflies]
    swarm.timeHelper.sleep(5.0)
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
