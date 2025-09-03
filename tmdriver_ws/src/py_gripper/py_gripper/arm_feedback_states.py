from tm_msgs.srv import SetPositions,SetEvent,SendScript
from tm_msgs.msg import FeedbackState
from robotiq_85_msgs.msg import GripperCmd
from py_gripper_interfaces.srv import Trajectory

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import time
import numpy as np
import queue
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped,PoseStamped

class ArmFeedbackStates(Node):
    def __init__(self):
        super().__init__('arm_feedback_states')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pos_sub = self.create_subscription(FeedbackState, 'feedback_states', self.pos_callback, 10)
        self.get_logger().info("Start Arm Feedback States")
        

    def pos_callback(self,msg):
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'arm'
        x,y,z = msg.tool_pose[:3]
        q = R.from_euler('xyz', msg.tool_pose[3:], degrees=False).as_quat()
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)


    
def main(args=None):
    rclpy.init(args=args)
    arm = ArmFeedbackStates()
    rclpy.spin(arm)
    rclpy.shutdown()