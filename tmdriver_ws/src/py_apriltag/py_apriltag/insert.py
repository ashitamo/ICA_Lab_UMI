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
from scipy.spatial.transform import Rotation as R
import time
import yaml
from hole_pos import fixedhole_A_C


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


# Load from YAML file
with open('ICA_Lab_UMI_Config.yaml', 'r') as file:
    config_data = yaml.safe_load(file)
    T_G_C = np.array(config_data['T_G_C'])
    T_C_G = np.array(config_data['T_C_G'])
    T_G_E = np.array(config_data['T_G_E'])
    T_E_G = np.array(config_data['T_E_G'])
    T_W_a = np.array(config_data['T_W_a'])
    T_a_W = np.linalg.inv(T_W_a)
    T_a_A = np.array(config_data['T_a_A'])
    T_A_a = np.array(config_data['T_A_a'])
    T_E_C = T_E_G @ T_G_C
    T_C_E = np.linalg.inv(T_E_C)

alpha = 0.2
lpf = [RealtimeLowPassFilter(alpha) for i in range(6)]
def calVelocity(p0,p1,p2):
    t0 = p0[0]
    t1 = p1[0]

    p0 = p0[1:]
    p1 = p1[1:]

    v = (p1[:3]-p0[:3])/(t1-t0)

    r0 = p0[3:]
    r1 = p1[3:]

    w = (r1 - r0)
    for i in range(3):
        if abs(w[i]) > np.pi:
            w[i] = w[i] - np.sign(w[i])*2*np.pi
    w = w/(t1-t0)

    v = np.append(v,w)

    v = [lpf[i].update(v[i]) for i in range(6)]
    return v

class InsertNode(Node):

    def __init__(self):
        super().__init__('insert_node')
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.path = Path()
        self.path.header.frame_id = 'world'
        self.target_pose_list_sub = self.create_subscription(Float64MultiArray, '/target_pose_list', self.target_pose_list_callback, qos_profile_sensor_data)

        self.target_pose_client = self.create_client(Trajectory, '/target_pose_service')
        # while not self.target_pose_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # print('target_pose_service service available')

        self.joyclient = self.create_client(Trajectory, '/trajectory')
        while not self.joyclient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        print('trajectory service available')

        self.req = Trajectory.Request()
        self.paths = []
        self.last_pos = None

        HZ = 10
        self.create_timer(1/HZ, self.pathplay)
        print('init done')
        
        self.gripLen = 0.0295
        self.initRot = [180.0, 0.0, 180.0]
        self.approach()
        self.insert()
        

    def target_pose_list_callback(self,msg):
        self.get_logger().info('I heard: %s' % msg.data)
        self.paths.append(msg.data)

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
        T_a_x = np.eye(4)
        T_a_x[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_a_x[:3,3] = np.array(p[1:4])
        T_W_x = T_W_a @ T_a_x
        p[1] = T_W_x[0,3]
        p[2] = T_W_x[1,3]
        p[3] = T_W_x[2,3]
        p[4:8] = R.from_matrix(T_W_x[:3,:3]).as_quat()
        return p
    
    def w2a(self,p):
        T_W_x = np.eye(4)
        T_W_x[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_W_x[:3,3] = np.array(p[1:4])
        T_a_x = T_a_W @ T_W_x
        p[1] = T_a_x[0,3]
        p[2] = T_a_x[1,3]
        p[3] = T_a_x[2,3]
        p[4:8] = R.from_matrix(T_a_x[:3,:3]).as_quat()
        return p
    
    def g2c(self,p):
        T_x_G = np.eye(4)
        T_x_G[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_x_G[:3,3] = np.array(p[1:4])
        T_x_C = T_x_G @ T_G_C
        p[1] = T_x_C[0,3]
        p[2] = T_x_C[1,3]
        p[3] = T_x_C[2,3]
        p[4:8] = R.from_matrix(T_x_C[:3,:3]).as_quat()
        return p
    
    def c2g(self,p):
        T_x_C = np.eye(4)
        T_x_C[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_x_C[:3,3] = np.array(p[1:4])
        T_x_G = T_x_C @ T_C_G
        p[1] = T_x_G[0,3]
        p[2] = T_x_G[1,3]
        p[3] = T_x_G[2,3]
        p[4:8] = R.from_matrix(T_x_G[:3,:3]).as_quat()
        return p

    def pathplay(self):
        if len(self.paths) == 0:
            return
        #p1_A_C
        p1 = self.paths.pop(0)
        p1 = np.array(p1)
        p1 = self.a2w(p1.copy())
        p1 = self.c2g(p1.copy())
        

        grip = p1[8]
        idx = int(p1[0])
        rot = R.from_quat(p1[4:8]).as_euler('xyz', degrees=False)
        p1 = np.array([p1[1],p1[2],p1[3],rot[0],rot[1],rot[2]])

        if self.last_pos is not None:
            mv = 0.06
            mw = 0.25
            dp = p1-self.last_pos
            for i in range(3,6):
                if abs(dp[i]) > np.pi:
                    dp[i] = dp[i] - np.sign(dp[i])*2*np.pi
            temp = np.concatenate([dp[:3]/mv, dp[3:]/mw])
            duration = np.max(np.abs(temp))
            if idx == -1: # stay
                duration = 1.0
                velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            else: # normal move
                pa = np.array([0.0, self.last_pos[0],self.last_pos[1],self.last_pos[2],self.last_pos[3],self.last_pos[4],self.last_pos[5]])
                pb = np.array([duration, p1[0],p1[1],p1[2],p1[3],p1[4],p1[5]])
                velocity = calVelocity(pa.copy(),pb.copy(),None)
        else:
            duration = 2.0
            velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.last_pos = p1
        
        position = p1.tolist()
        self.get_logger().info('')
        self.get_logger().info(f'idx:{idx}')
        self.get_logger().info(f'duration:{duration}')
        self.get_logger().info(f'position:{position}')
        self.get_logger().info(f'velocity:{velocity}')
        self.get_logger().info(f'grip:{grip}')
        assert duration >= 0

        res = self.trajectoryRequest(
            idx=idx,
            positions=position,
            duration=duration,
            velocity=velocity,
            grip=grip
        )
    
    def target_pose_request(self,positions):
        self.paths.append(positions)

        # req = Trajectory.Request()
        # req.idx = int(positions[0])
        # req.positions = positions[1:8]
        # req.grip = positions[-1]
        # future = self.target_pose_client.call_async(req)
        # return future.result()

    def approach(self):
        init_pos_W_G = np.array([0.0,-0.0855, -0.5165, 0.37, 0.0, 0.0, 0.0, 1.0, 0.85])
        init_pos_W_G[4:8] = R.from_euler('xyz', self.initRot, degrees=True).as_quat()

        init_pos_A_G = self.w2a(init_pos_W_G.copy())
        init_pos_A_C = self.g2c(init_pos_A_G.copy())
        self.target_pose_request(init_pos_A_C.tolist())

        cylinderPos_W_G = np.array([1.0,-0.297, -0.525, 0.245, 0.0, 0.0, 0.0, 1.0, self.gripLen])
        cylinderPos_W_G[4:8] = R.from_euler('xyz', self.initRot, degrees=True).as_quat()
        cylinderPos_A_G = self.w2a(cylinderPos_W_G.copy())
        cylinderPos_A_C = self.g2c(cylinderPos_A_G.copy())

        temp = cylinderPos_A_C.copy()
        temp[3] += 0.1
        temp[-1] = 0.85
        self.target_pose_request(temp.tolist()) #cyc up
        cylinderPos_A_C[0] = 2.0
        self.target_pose_request(cylinderPos_A_C.tolist()) #cyc down

        cylinderPos_A_C[0] = -1.0
        self.target_pose_request(cylinderPos_A_C.tolist()) #grasp
        
        temp = cylinderPos_A_C.copy()
        temp[0] = 0.0
        temp[3] += 0.1
        self.target_pose_request(temp.tolist()) #cyc up

    def approach_G(self):
        init_pos_W_G = np.array([0.0,-0.0855, -0.5165, 0.37, 0.0, 0.0, 0.0, 1.0, 0.85])
        init_pos_W_G[4:8] = R.from_euler('xyz', self.initRot, degrees=True).as_quat()

        self.target_pose_request(init_pos_W_G.tolist())

        cylinderPos_W_G = np.array([1.0,-0.297, -0.525, 0.25, 0.0, 0.0, 0.0, 1.0, self.gripLen])
        cylinderPos_W_G[4:8] = R.from_euler('xyz', self.initRot, degrees=True).as_quat()

        temp = cylinderPos_W_G.copy()
        temp[3] += 0.1
        temp[-1] = 0.85
        self.target_pose_request(temp.tolist()) #cyc up
        cylinderPos_W_G[0] = 2.0
        self.target_pose_request(cylinderPos_W_G.tolist()) #cyc down

        cylinderPos_W_G[0] = -1.0
        self.target_pose_request(cylinderPos_W_G.tolist()) #grasp

        temp = cylinderPos_W_G.copy()
        temp[0] = 0.0
        temp[3] += 0.1
        self.target_pose_request(temp.tolist()) #cyc up
        

    def insert(self):
        # holePos_W_G = np.array([0.0,-0.115, -0.515, 0.345, 0.0, 0.0, 0.0, 1.0, self.gripLen])
        # holePos_W_G[4:8] = R.from_euler('xyz', self.initRot, degrees=True).as_quat()
        holePos_A_C = fixedhole_A_C['bottom_left'].copy()
        holePos_W_C = self.c2g(holePos_A_C.copy())
        holePos_W_G = self.a2w(holePos_W_C.copy())

        temp = holePos_A_C.copy()
        temp[3] += 0.05
        self.target_pose_request(temp.tolist())
        holePos_A_C[0] = 1.0
        self.target_pose_request(holePos_A_C.tolist()) # pretouch
        # self.target_pose_request(temp.tolist()) # up

        # lastPos_W_G = np.array([-1.0,-0.115, -0.45, 0.4, 0.0, 0.0, 0.0, 1.0, self.gripLen])
        # lastPos_W_G[4:8] = R.from_euler('xyz', self.initRot, degrees=True).as_quat()
        # lastPos_A_G = self.w2a(lastPos_W_G.copy())
        # lastPos_A_C = self.g2c(lastPos_A_G.copy())
        # self.target_pose_request(lastPos_A_C.tolist())

        # holePosApproach, holePosTouch ,holePosFinal = self.plan_insert_traj(holePos_W_G,lastPos_W_G)
        # holePosApproach = self.w2a(holePosApproach.copy())
        # holePosApproach = self.g2c(holePosApproach.copy())

        # holePosTouch = self.w2a(holePosTouch.copy())
        # holePosTouch = self.g2c(holePosTouch.copy())

        # holePosFinal = self.w2a(holePosFinal.copy())
        # holePosFinal = self.g2c(holePosFinal.copy())

        # holePosApproach[0] = 0.0
        # self.target_pose_request(holePosApproach.tolist())
        # holePosTouch[0] = 1.0
        # self.target_pose_request(holePosTouch.tolist())
        # holePosFinal[0] = 2.0
        # self.target_pose_request(holePosFinal.tolist())
        # holePosFinal[0] = 3.0
        # holePosFinal[3]-= 0.03
        # holePosFinal[-1] = 0.085
        # self.target_pose_request(holePosFinal.tolist())
        # holePosFinal[0] = 4.0
        # holePosFinal[3] = 0.35
        # self.target_pose_request(holePosFinal.tolist())

        # print(holePos_W_G)
        # print(holePosApproach)
        # print(holePosTouch)
        # print(holePosFinal)
        # print()
    
    

    def plan_insert_traj(self,holePos_W_G,seeholePos_W_G,r=0.1,th1=15.0,cylinderLen=0.09, c = 0.002,b=0.0):
        holePos_W_P = self.g2p(holePos_W_G,cylinderLen)

        x0,y0,z0 = seeholePos_W_G[1],seeholePos_W_G[2],seeholePos_W_G[3]
        x2,y2,z2 = holePos_W_P[1],holePos_W_P[2],holePos_W_P[3]
        
        th2 = math.atan2(x2 - x0, y2 - y0)
        th1 = math.radians(th1)

        print(th1,th2)

        rz = -math.degrees(th2)
        rx = math.degrees(th1)
        print(rx,-rz)
        l = r*math.sin(th1)
        x1,y1,z1 = -l*math.sin(th2)+x2,  -l*math.cos(th2)+y2,  r*math.cos(th1)+z2

        holePos_W_P_approch = np.array([0.0, x1, y1, z1, 0.0, 0.0, 0.0, 1.0, self.gripLen])
        holePos_W_P_approch[4:8] = R.from_euler('xyz', [180.0+rx, 0.0, rz], degrees=True).as_quat()

        holePos_W_P[4:8] = R.from_euler('xyz', [180.0+rx, 0.0, rz], degrees=True).as_quat()

        holePos_W_G_approch = self.p2g(holePos_W_P_approch,cylinderLen)
        holePos_W_G_touch = self.p2g(holePos_W_P,cylinderLen)

        print(l)
        print()
        holePos_W_G_final = holePos_W_G_touch.copy()
        holePos_W_G_final[1] = (l+c)*math.sin(th2)+holePos_W_G_touch[1]
        holePos_W_G_final[2] = (l+c)*math.cos(th2)+holePos_W_G_touch[2]
        holePos_W_G_final[3] = holePos_W_G_touch[3] - 0.04
        holePos_W_E_final = self.g2e(holePos_W_G_final)
        holePos_W_E_final[4:8] = R.from_euler('xyz', [180.0, 0.0, rz], degrees=True).as_quat()
        holePos_W_G_final = self.e2g(holePos_W_E_final)

        return holePos_W_G_approch, holePos_W_G_touch ,holePos_W_G_final
    
    def insert_G(self):
        holePos_W_G = np.array([0.0,-0.115, -0.515, 0.345, 0.0, 0.0, 0.0, 1.0, self.gripLen])
        holePos_W_G[4:8] = R.from_euler('xyz', self.initRot, degrees=True).as_quat()
        
        temp = holePos_W_G.copy()
        temp[3] += 0.05
        self.target_pose_request(temp.tolist())
        holePos_W_G[0] = 1.0
        self.target_pose_request(holePos_W_G.tolist()) # pretouch
        self.target_pose_request(temp.tolist()) # up

        lastPos_W_G = np.array([-1.0,-0.115, -0.45, 0.4, 0.0, 0.0, 0.0, 1.0, self.gripLen])
        lastPos_W_G[4:8] = R.from_euler('xyz', self.initRot, degrees=True).as_quat()
        self.target_pose_request(lastPos_W_G.tolist())
    
        holePosApproach, holePosTouch ,holePosFinal = self.plan_insert_traj(holePos_W_G,lastPos_W_G)
        holePosApproach[0] = 0.0
        self.target_pose_request(holePosApproach.tolist())
        holePosTouch[0] = -1.0
        self.target_pose_request(holePosTouch.tolist())
        holePosFinal[0] = -1.0
        self.target_pose_request(holePosFinal.tolist())
        holePosFinal[0] = -1.0
        holePosFinal[3]-= 0.03
        holePosFinal[-1] = 0.085
        self.target_pose_request(holePosFinal.tolist())
        holePosFinal[3] = 0.35
        self.target_pose_request(holePosFinal.tolist())

        print(holePos_W_G)
        print(holePosApproach)
        print(holePosTouch)
        print(holePosFinal)
        print()

    def p2g(self,p,len):
        T_W_P = np.eye(4)
        T_W_P[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_W_P[:3,3] = np.array(p[1:4])
        T_P_E = np.array(
            [[ 1, 0, 0, 0],
            [ 0, 1, 0, 0],
            [ 0, 0, 1, -len],
            [ 0, 0, 0, 1]]
        )
        T_W_G = T_W_P @ T_P_E @ T_E_G
        p[1] = T_W_G[0,3]
        p[2] = T_W_G[1,3]
        p[3] = T_W_G[2,3]
        p[4:8] = R.from_matrix(T_W_G[:3,:3]).as_quat()
        return p

    def g2p(self,p,len):
        T_W_G = np.eye(4)
        T_W_G[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_W_G[:3,3] = np.array(p[1:4])
        T_E_P = np.array(
            [[ 1, 0, 0, 0],
            [ 0, 1, 0, 0],
            [ 0, 0, 1, len],
            [ 0, 0, 0, 1]]
        )
        T_W_E = T_W_G @ T_G_E @ T_E_P
        p[1] = T_W_E[0,3]
        p[2] = T_W_E[1,3]
        p[3] = T_W_E[2,3]
        p[4:8] = R.from_matrix(T_W_E[:3,:3]).as_quat()
        return p
    
    def g2e(self,p):
        T_x_G = np.eye(4)
        T_x_G[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_x_G[:3,3] = np.array(p[1:4])
        T_x_E = T_x_G @ T_G_E
        p[1] = T_x_E[0,3]
        p[2] = T_x_E[1,3]
        p[3] = T_x_E[2,3]
        p[4:8] = R.from_matrix(T_x_E[:3,:3]).as_quat()
        return p
    
    def e2g(self,p):
        T_x_E = np.eye(4)
        T_x_E[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_x_E[:3,3] = np.array(p[1:4])
        T_x_G = T_x_E @ T_E_G
        p[1] = T_x_G[0,3]
        p[2] = T_x_G[1,3]
        p[3] = T_x_G[2,3]
        p[4:8] = R.from_matrix(T_x_G[:3,:3]).as_quat()
        return p

def main():
    rclpy.init()
    node = InsertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
