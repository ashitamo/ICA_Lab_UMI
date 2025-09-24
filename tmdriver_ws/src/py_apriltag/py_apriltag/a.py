import math

from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped,PoseStamped
from std_msgs.msg import Float64MultiArray
from py_gripper_interfaces.srv import Trajectory

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from dt_apriltags import Detector
import os
import cv2
from scipy.spatial.transform import Rotation as R
import time


class FramePublisher(Node):

    def __init__(self):
        super().__init__('test_pub')
        self.pub = self.create_publisher(Float64MultiArray, '/target_pose_list', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.i = 0
    def timer_callback(self):
        self.i = self.i % 2
        msg = Float64MultiArray()
        msg.data = [float(self.i),0.0,0.0,0.0, 0.0,0.0,0.0,0.5, 0.0]
        self.pub.publish(msg)
        self.get_logger().info('Publishing: %s' % msg.data)
        self.i += 1

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
