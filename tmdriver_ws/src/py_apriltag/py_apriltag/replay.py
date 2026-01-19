import math

from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped,PoseStamped
from std_msgs.msg import Float64MultiArray
from py_gripper_interfaces.srv import Trajectory
from py_gripper_interfaces.msg import TrajState

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

import yaml

class FramePublisher(Node):

    def __init__(self):
        super().__init__('test_publisher')
        # Initialize the transform broadcaster
        self.action_clinet = self.create_client(Trajectory,'/target_pose_service')
        self.subscription= self.create_subscription(
            TrajState,
            '/traj_state',
            self.data_callback,
            qos_profile_sensor_data
        )
        
        while not self.action_clinet.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        print("Service connected!")
        HZ = 10
        
        print('init done')
        with open('/home/lab606/ICA_Lab_UMI/UMI_exe_12/rec_traj.txt', 'r') as f:
            self.lines = f.readlines()
        self.idx = 0
        # self.create_timer(1/HZ, self.pathplay)
        self.data = []
        self.pathplay()

    def data_callback(self, msg):
        # img = self.bridge.imgmsg_to_cv2(msg.color_image, desired_encoding='bgr8')
        idx = msg.idx
        # depth = self.bridge.imgmsg_to_cv2(msg.depth_image, desired_encoding='passthrough')
        self.data.append([idx,None,None])
        print(f"Get feedback id={idx} from /traj_state topic")

    def action_publish(self, command_list: list):
        """Takes an 8-element list, creates a Float32MultiArray message, and publishes it."""
        if len(command_list) != 9:
            self.get_logger().error(f"Invalid command list length. Expected 9, got {len(command_list)}.")
            return
        
        msg = Float64MultiArray()
        msg.data = [float(p) for p in command_list]
        # msg.data = array('d', [-0.185891, 0.068578, 0.398634, -0.591989, 0.685533, -0.340521, 0.252266, 0.165614])
        # msg = std_msgs.msg.Float64MultiArray(layout=std_msgs.msg.MultiArrayLayout(dim=[], data_offset=0), data=[-0.185891, 0.068578, 0.398634, -0.591989, 0.685533, -0.340521, 0.252266, 0.165614])
        
        id = command_list[0]
        self.get_logger().info(f"Publishing: Pose={msg.data} with ID={id}")
        # self.pose_publisher.publish(msg)
        req = Trajectory.Request()
        req.positions = msg.data[1:8]
        req.idx = int(msg.data[0])
        req.grip = msg.data[-1]
        self.action_clinet.call_async(req)
        self.wait_feedback_count = 0    

    def wait_feedback(self, wait_for_id):
        while True:
            rclpy.spin_once(self,timeout_sec=0.1)
            if len(self.data):
                field_data = self.data[0]
                if field_data[0] == wait_for_id:
                    print("Confirm action finish!")
                    self.data.pop(0)
                    return field_data[1], field_data[2]
                else:
                    self.data.pop(0)
                    print(f"id={field_data[0]}, Call back id not as expected")
            else:
                # print("Waiting task finish and return...")
                self.wait_feedback_count += 1
                if self.wait_feedback_count > 50:
                    print("No feedback received for a long time, proceeding anyway.")
                    return None, None
                pass

    def pathplay(self):
        while self.idx < len(self.lines):
            line = self.lines[self.idx].strip().split(',')
            point = [float(x) for x in line[:]]
            self.action_publish(point)
            id = int(point[0])
            if id == -1:
                self.wait_feedback(-1)
                input("Press Enter to continue...")
            print("published: ",point)
            self.idx += 1
            # rclpy.spin_once(self, timeout_sec=0.1)
        print("Path playback completed.")
        

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
