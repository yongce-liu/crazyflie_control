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
