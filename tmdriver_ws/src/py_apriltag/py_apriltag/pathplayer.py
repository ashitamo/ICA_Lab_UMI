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
class RealtimeLowPassFilter:
    def __init__(self, alpha):
        """
        Initializes a first-order low-pass filter.
        :param alpha: Filter coefficient, between 0 and 1.
                      Higher alpha means more smoothing (lower cutoff frequency).
        """
        if not (0 <= alpha <= 1):
            raise ValueError("Alpha must be between 0 and 1.")
        self.alpha = alpha
        self.filtered_value = 0.0

    def update(self, new_sample):
        """
        Processes a new sample and updates the filtered value.
        :param new_sample: The new data point to filter.
        :return: The current filtered value.
        """
        self.filtered_value = self.alpha * new_sample + (1 - self.alpha) * self.filtered_value
        return self.filtered_value
    
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

camera_params = [839.72913173,835.51579792,311.87321929,255.38762288]
# camera_params = [839.72913173*2,835.51579792*2,311.87321929*2,255.38762288*2]

# 170 -415 190 mm 180 0 -90
T_W_W = np.eye(4)
T_W_A = np.array([[ 0, 1, 0, 0.170],
                  [ 1, 0, 0 ,-0.315],
                  [ 0, 0, -1, 0.04],
                  [ 0, 0,  0 ,1]])

T_A_W = np.linalg.inv(T_W_A)


    
T_a_A =  np.array([[ 0, 1, 0, 0],
                  [ 1, 0, 0 ,0],
                  [ 0, 0, -1, 0],
                  [ 0, 0,  0 ,1]])
T_A_a = np.linalg.inv(T_a_A)

T_W_a = T_W_A @ T_A_a

T_W_a = np.array(
        [[ 0.02978282, 0.99917894,-0.02746683,-0.35052857],
        [-0.99919005, 0.03050458, 0.0262435 ,-0.72868842],
        [ 0.02705982, 0.02666297, 0.99927817, 0.01466152],
        [ 0.        , 0.        , 0.        , 1.        ]]
)
print(T_W_a)
T_C_c = np.array([[ -1, 0, 0, 0],
                  [ 0, 1, 0 ,0],
                  [ 0, 0, -1 ,0],
                  [ 0, 0,  0 ,1]])

T_c_C = np.linalg.inv(T_C_c)

T_G_c = np.array([[ 0.99779789, -0.06619949, -0.00412319, -0.00555809],
                [ 0.06559438,  0.97563804,  0.20935089, -0.06929543],
                [-0.00983618, -0.20916034,  0.97783189,  0.03819794],
                [ 0.0,          0.0,          0.0,      1.0    ]]
)
T_c_G = np.linalg.inv(T_G_c)
alpha = 0.2
lpf = [RealtimeLowPassFilter(alpha) for i in range(6)]
def calVelocity(p0,p1,p2):
    t0 = p0[0]
    t2 = p2[0]

    p0 = p0[1:]
    p2 = p2[1:]

    v = (p2[:3]-p0[:3])/(t2-t0)

    r0 = p0[3:]
    r2 = p2[3:]

    w = (r2 - r0)
    for i in range(3):
        if abs(w[i]) > np.pi:
            w[i] = w[i] - np.sign(w[i])*2*np.pi
    w = w/(t2-t0)

    v = np.append(v,w)

    v = [lpf[i].update(v[i]) for i in range(6)]
    return v

class FramePublisher(Node):

    def __init__(self):
        super().__init__('pose_publisher')
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.path = Path()
        self.path.header.frame_id = 'world'
        self.publisher_ = self.create_publisher(Path, '/cam_path', 10)
        self.joy_publisher = self.create_publisher(Float64MultiArray, '/joy', qos_profile_sensor_data)

        self.joyclient = self.create_client(Trajectory, '/trajectory')
        # while not self.joyclient.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        print('service available')

        self.req = Trajectory.Request()
        self.lines = open('circle.txt', 'r').readlines()
        for i in range(len(self.lines)):
            g = 0.0
            line = self.lines[i].split(',')
            if len(line) == 9:
                stamp,x,y,z,px,py,pz,pw,g = [float(x) for x in line]
            else:
                stamp,x,y,z,px,py,pz,pw = [float(x) for x in line]
            self.lines[i] = [stamp,x,y,z,px,py,pz,pw,g]

        self.i = 0
        HZ = 90
        self.create_timer(1/HZ, self.pathplay)
        print('init done')

    def trajectoryRequest(self,idx,positions,duration,velocity=[0.0,0.0,0.0,0.0,0.0,0.0],grip = 0.0,wait=False):
        
        self.req.mode = Trajectory.Request.PATH
        self.req.idx = idx
        self.req.positions = list(map(float,positions))
        self.req.velocity = list(map(float,velocity))
        self.req.duration = float(duration)
        self.req.grip = float(grip)

        self.future = self.joyclient.call_async(self.req)
        if wait:
            rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()


    def a2w(self,p):
        T_a_ci = np.eye(4)
        T_a_ci[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_a_ci[:3,3] = np.array(p[1:4])
        T_W_gi = T_W_a @ T_a_ci @ T_c_G
        p[1] = T_W_gi[0,3]
        p[2] = T_W_gi[1,3]
        p[3] = T_W_gi[2,3]
        p[4:8] = R.from_matrix(T_W_gi[:3,:3]).as_quat()
        return p
    
    def pathplay(self):
        if self.i >= len(self.lines):
            return
        
        p0 = self.a2w(self.lines[(self.i-1)%len(self.lines)].copy())
        p1 = self.a2w(self.lines[self.i%len(self.lines)].copy())
        p2 = self.a2w(self.lines[(self.i+1)%len(self.lines)].copy())
        if self.i % len(self.lines) == 0:
            self.i = 0

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'april_tag'
        q = R.from_matrix(T_W_a[:3,:3]).as_quat()
        t.transform.translation.x = float(T_W_a[0,3])
        t.transform.translation.y = float(T_W_a[1,3])
        t.transform.translation.z = float(T_W_a[2,3])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'gripper'
        t.transform.translation.x = p1[1]
        t.transform.translation.y = p1[2]
        t.transform.translation.z = p1[3]
        t.transform.rotation.x = p1[4]
        t.transform.rotation.y = p1[5]
        t.transform.rotation.z = p1[6]
        t.transform.rotation.w = p1[7]

        grip = p1[8]
        rot = R.from_quat(p0[4:8]).as_euler('xyz', degrees=False)
        p0 = np.array([p0[0],p0[1],p0[2],p0[3],rot[0],rot[1],rot[2]])
        rot = R.from_quat(p1[4:8]).as_euler('xyz', degrees=False)
        p1 = np.array([p1[0],p1[1],p1[2],p1[3],rot[0],rot[1],rot[2]])
        rot = R.from_quat(p2[4:8]).as_euler('xyz', degrees=False)
        p2 = np.array([p2[0],p2[1],p2[2],p2[3],rot[0],rot[1],rot[2]])
        
        velocity = [0.0,0.0,0.0,0.0,0.0,0.0]
        if self.i % len(self.lines) != 0:
            velocity = calVelocity(p0.copy(),p1.copy(),p2.copy())    
            duration = p1[0] - p0[0]
        else:
            duration = 2
        position = [p1[1],p1[2],p1[3],p1[4],p1[5],p1[6]]
        self.get_logger().info('')
        self.get_logger().info(f'duration:{duration}')
        self.get_logger().info(f'position:{position}')
        self.get_logger().info(f'velocity:{velocity}')
        self.get_logger().info(f'grip:{grip}')
        assert duration > 0

        self.tf_broadcaster.sendTransform(t)
        res = self.trajectoryRequest(
            idx=self.i,
            positions=position,
            duration=duration,
            velocity=velocity,
            grip=grip
        )
        self.publish_path(t)
        self.i += 1

    def publish_path(self,t: TransformStamped):
        pose = PoseStamped()
        pose.header.frame_id = t.child_frame_id
        pose.header.stamp = t.header.stamp
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z

        pose.pose.orientation.x = t.transform.rotation.x
        pose.pose.orientation.y = t.transform.rotation.y
        pose.pose.orientation.z = t.transform.rotation.z
        pose.pose.orientation.w = t.transform.rotation.w
        self.path.poses.append(pose)
        if len(self.path.poses) > 1000:
            self.path.poses.pop(0)
        self.publisher_.publish(self.path)
        # self.get_logger().info(f'Publishing path with {len(self.path.poses)} poses')

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
