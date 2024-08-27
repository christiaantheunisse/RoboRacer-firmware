import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
import math
from typing import Optional, Literal, TypeAlias
import time

from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from racing_bot_control.lib.control_node import VelocityControlNode, ControlMode


def main(args=None):
    rclpy.init(args=args)

    velocity_control_node = VelocityControlNode()

    rclpy.spin(velocity_control_node)
    velocity_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
