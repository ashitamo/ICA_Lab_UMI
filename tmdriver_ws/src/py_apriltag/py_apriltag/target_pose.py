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
from py_apriltag.smooth_pvt import SmoothPVT,_unwrap_euler,moving_average_position_and_orientation
import os
import yaml

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
with open(os.path.join(os.environ['HOME'], 'ICA_Lab_UMI/ICA_Lab_UMI_Config.yaml'), 'r') as file:
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
lpf_pos = [RealtimeLowPassFilter(0.1) for i in range(6)]
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

class TargetPoseNode(Node):
    def __init__(self):
        super().__init__('target_pose')

        self.joy_publisher = self.create_publisher(Float64MultiArray, '/joy', qos_profile_sensor_data)
        self.target_pose_list_sub = self.create_subscription(Float64MultiArray, '/target_pose_list', self.target_pose_list_callback, qos_profile_sensor_data)
        self.target_pose_list_sev = self.create_service(Trajectory, '/target_pose_service', self.target_pose_service_callback)


        self.joyclient = self.create_client(Trajectory, '/trajectory')
        while not self.joyclient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('trajectory service available')

        self.req = Trajectory.Request()
        self.paths = []
        self.last_pos = None
        self.pvt = SmoothPVT(
            v_lin=0.05,   # 先稍微再慢一點
            v_ang=0.7,
            a_lin=1.5,
            a_ang=2.5,
            a_c_max=0.7,  # 轉彎更保守：向心加速度上限
            min_T=0.008,   # 每段至少 0.008s
            tau_v=0.50    # 速度濾波時間常數
        )

        self.last_paths = []
        HZ = 30
        self.create_timer(1/HZ, self.pathplay)
        self.get_logger().info('init done')

    def target_pose_list_callback(self,msg):
        self.get_logger().info('I heard: %s' % msg.data)
        self.paths.append(msg.data)

    def target_pose_service_callback(self,req:Trajectory.Request,res:Trajectory.Response):
        # self.get_logger().info('Service called')
        pos = [req.idx, 
               req.positions[0], req.positions[1], req.positions[2],
               req.positions[3], req.positions[4], req.positions[5],req.positions[6],
                req.grip]
        self.paths.append(pos)
        res.ok = True
        return res

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
        if len(self.last_paths)>=14:
            p1 = np.array(self.paths.pop(0))
            p1 = self.a2w(p1.copy())
            p1 = self.c2g(p1.copy())
            grip = float(p1[8])
            idx = int(p1[0])
            self.last_paths.append((None,np.array([p1[1], p1[2], p1[3], p1[4], p1[5], p1[6], p1[7]])))
            # p1 = moving_average_position_and_orientation(self.last_paths, window_size=15)
            # p1 = np.array([idx, p1[0], p1[1], p1[2], p1[3], p1[4], p1[5], p1[6], grip])
            self.last_paths.pop(0)
        else:
            # 取出當前目標（你的座標轉換保持不變）
            p1 = np.array(self.paths.pop(0))
            p1 = self.a2w(p1.copy())
            p1 = self.c2g(p1.copy())
            grip = float(p1[8])
            idx = int(p1[0])
            self.last_paths.append((None,np.array([p1[1], p1[2], p1[3], p1[4], p1[5], p1[6], p1[7]])))

        # 位置與姿態
        p1_xyz = np.array([p1[1], p1[2], p1[3]])
        q1_xyzw = R.from_quat(p1[4:8]).as_quat()  # xyzw

        # lookahead（若有）
        if len(self.paths) > 0:
            p2 = np.array(self.paths[0])
            p2 = self.a2w(p2.copy())
            p2 = self.c2g(p2.copy())
            p2_xyz = np.array([p2[1], p2[2], p2[3]])
            q2_xyzw = R.from_quat(p2[4:8]).as_quat()
        else:
            p2_xyz, q2_xyzw = None, None

        # 初次進來時，假定上一點是自己（零段長），並令初速為 0
        if self.last_pos is None:
            self.last_pos = np.concatenate([p1_xyz, R.from_quat(p1[4:8]).as_euler('xyz')])
            self.pvt.last_v6[:] = 0.0
            duration = 1.5
            velocity = [0.0]*6
        else:
            # 上一個命令點（世界座標，取你保存的 last_pos）
            p0_xyz = np.array(self.last_pos[:3])
            q0_xyzw = R.from_euler('xyz', self.last_pos[3:], degrees=False).as_quat()
            if abs(p1_xyz - p0_xyz).sum() < 1e-6 and abs(q1_xyzw - q0_xyzw).sum() < 1e-6:
                return

            # 計算段時間與末端速度（定速＋拐角平滑＋加速度限幅）
            duration, v_end = self.pvt.plan_segment(
                p0_xyz, q0_xyzw, p1_xyz, q1_xyzw, p2_xyz, q2_xyzw
            )
            velocity = v_end.tolist()

        # 以歐拉回填 positions（你的下游介面吃 [x,y,z,rx,ry,rz]）
        rot_xyz_new = R.from_quat(q1_xyzw).as_euler('xyz', degrees=False)
        if self.pvt.last_euler is None:
            rot_xyz = rot_xyz_new
        else:
            rot_xyz = _unwrap_euler(self.pvt.last_euler, rot_xyz_new)
        self.pvt.last_euler = rot_xyz.copy()
        position = [p1_xyz[0], p1_xyz[1], p1_xyz[2], rot_xyz[0], rot_xyz[1], rot_xyz[2]]


        self.last_pos = np.array(position)

        # 若 idx == -1 / 0 要求停住，可覆蓋為零速與固定小時間
        if idx in (-1, 0) or len(self.paths) == 0:
            duration = 2.0
            velocity = [0.0]*6
            self.last_pos = None
            self.last_paths = []

        self.get_logger().info(f'idx:{idx}')
        self.get_logger().info(f'duration:{duration}')
        self.get_logger().info(f'position:{position}')
        self.get_logger().info(f'velocity:{velocity}')
        self.get_logger().info(f'grip:{grip}')
        assert duration >= 0

        self.trajectoryRequest(
            idx=idx,
            positions=position,
            duration=duration,
            velocity=velocity,
            grip=grip
        )
    
    def pathplay_copy(self):
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
            if abs(p1-self.last_pos).sum() < 1e-6:
                return
            mv = 0.05
            if len(self.paths) <10:
                mv = len(self.paths)*(0.5-0.1)/10 + 0.1
            p0 = self.last_pos
            d = np.linalg.norm(p1[:3] - p0[:3])
            duration = d/mv
            v = (p1[:3] - p0[:3])/duration
            w = (p1[3:] - p0[3:])
            for j in range(len(w)):
                if w[j] > np.pi:
                    w[j] -= 2*np.pi
                elif w[j] < -np.pi:
                    w[j] += 2*np.pi
            w = w/duration
            velocity = np.array([v[0], v[1], v[2], w[0], w[1], w[2]])
            for j in range(len(velocity)):
                velocity[j] = lpf[j].update(velocity[j])
            if duration <=0.0:
                return
            if len(self.paths) == 0:
                velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            if idx == -1 or idx == 0: # stay
                duration = 1.5
                velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.last_pos = None
        else:
            duration = 1.5
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


def main():
    rclpy.init()
    node = TargetPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
