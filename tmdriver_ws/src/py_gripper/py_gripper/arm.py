from tm_msgs.srv import SetPositions,SetEvent,SendScript,SetIO
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

class Arm(Node):
    def __init__(self):
        super().__init__('arm')
        self.pos_cli = self.create_client(SetPositions, 'set_positions')
        self.event_cli = self.create_client(SetEvent, 'set_event')
        self.script_cli = self.create_client(SendScript, 'send_script')
        self.set_io_cli = self.create_client(SetIO, 'set_io')
        while not self.pos_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.event_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.script_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        while not self.set_io_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.gripper_pub = self.create_publisher(GripperCmd, '/gripper/cmd', 10)

        self.target_positions = np.array([0.0, -0.4, 0.35, 3.14159, 0.0, 3.14159])
        self.current_positions = np.array([0.0, -0.415, 0.35, 3.14159, 0.0, 3.14159])
        self.project_speed = 5

        self.delay_queue_size = 25
        self.cmd_queue = queue.Queue(150)
        for _ in range(self.delay_queue_size):
            self.cmd_queue.put(None)
        self.joy_queue = queue.Queue(25)

        self.pos_sub = self.create_subscription(FeedbackState, 'feedback_states', self.pos_callback, 10)
        # self.send_gripper(0.085)
        self.send_event()
        response = self.send_request(velocity=0.1,acc_time=0.1)
        while not self.is_arrived():
            rclpy.spin_once(self)
        print('arrived')

        self.joy_sub = self.create_subscription(Float64MultiArray, 'joy', self.joy_callback, qos_profile_sensor_data)
        self.gripList = []
        self.noCmdCount = 0
        self.PVTEnter()
        self.create_timer(0.01, self.run)
        self.create_timer(0.01, self.cmd_putter)
        self.create_timer(0.0025, self.gripperTimer)

        self.create_service(Trajectory, 'trajectory', self.trajectory_callback)
        print('init done')
        
    def trajectory_callback(self,req:Trajectory.Request,res:Trajectory.Response):
        self.joy_queue.put(req)
        res.ok = True
        # self.get_logger().info("Get Trajectory Request: %s" % req)
        return res


    def cmd_putter(self):
        if self.joy_queue.empty():
            self.cmd_queue.put(None)
        else:
            cmd = self.joy_queue.get()
            self.cmd_queue.put(cmd)

    def pos_callback(self,msg:FeedbackState):
        self.current_positions = msg.tool_pose
        self.project_speed = msg.project_speed
        # self.get_logger().info("Current Position: %s" % self.current_positions)
    

    def joy_callback(self,msg):
        self.joy_queue.put(msg.data)
        self.get_logger().info("Get Joy: %s" % msg.data)


    def is_arrived(self,error=0.01):
        if sum((self.target_positions[i]-self.current_positions[i])**2 for i in range(3)) > error**2:
            return False
        return True

    def gripperTimer(self):
        if len(self.gripList) == 0:
            return
        la,du,grip = self.gripList[0]
        if (time.time() - la) >= (du/self.project_speed*100.0):
            self.gripList.pop(0)
            self.send_gripper(grip)
            self.get_logger().info(f"Send Gripper: {du/self.project_speed*100.0} {grip}")
            if len(self.gripList) == 0:
                return
            self.gripList[0][0] = time.time()
    def PVTEnter(self):
        self.send_script("PVTEnter(1)")


    def PVTPoint(self,postions=np.array([0.2, -0.4, 0.35, 3.14159, 0.0, 3.14159]),
                 velocity=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                 duration=1):
        self.target_positions = postions.copy()
        postions[:3] = postions[:3]*1000
        postions[3:] = postions[3:]*180/np.pi
        postions = ",".join(str(x) for x in postions)
        velocity[:3] = velocity[:3]*1000
        velocity[3:] = velocity[3:]*180/np.pi
        velocity = ",".join(str(x) for x in velocity)
        # print("PVTPoint({}, {}, {})".format(postions, velocity, duration))
        self.send_script("PVTPoint({}, {}, {})".format(postions, velocity, duration))

    def PVTExit(self):
        self.send_script("PVTExit()")

    def set_io(self,io=0,value=0):
        req = SetIO.Request()
        req.module = SetIO.Request.MODULE_CONTROLBOX
        req.type = SetIO.Request.TYPE_ANALOG_OUT
        req.pin = 0
        req.state = value
        future = self.set_io_cli.call_async(req)
        return future.result()

    def send_request(self,positions=np.array([-0.12, -0.52, 0.35, 3.14159, 0.0, 3.14159]),
                     velocity=0.2, acc_time=0.05, blend_percentage=100, fine_goal=False):
        self.target_positions = positions
        print(self.target_positions)
        positions = list(positions)
        set_positions_req = SetPositions.Request()
        set_positions_req.motion_type = SetPositions.Request.LINE_T
        set_positions_req.positions = positions
        set_positions_req.velocity = velocity
        set_positions_req.acc_time = acc_time
        set_positions_req.blend_percentage = blend_percentage
        set_positions_req.fine_goal = fine_goal
        future = self.pos_cli.call_async(set_positions_req)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_gripper(self,gap=0.085):
        gap = gap if gap < 0.085 else 0.085
        gap = gap if gap > 0.0 else 0.0

        grip_msg = GripperCmd()
        grip_msg.emergency_release = False
        grip_msg.emergency_release_dir = 0
        grip_msg.stop = False
        grip_msg.position = gap
        grip_msg.speed = 1.0
        grip_msg.force = 10.0
        self.gripper_pub.publish(grip_msg)

    def send_event(self):
        set_event_req = SetEvent.Request()
        set_event_req.func = SetEvent.Request.STOP
        set_event_req.arg0 = 0
        set_event_req.arg1 = 0
        future = self.event_cli.call_async(set_event_req)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_script(self,cmd):
        script_req =  SendScript.Request()
        script_req.id = "arm"
        script_req.script = cmd
        future = self.script_cli.call_async(script_req)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def run(self):
        if not self.cmd_queue.empty():
            cmd = self.cmd_queue.get()
            # print(self.cmd_queue.qsize())
            if cmd is not None:
                if cmd.mode == Trajectory.Request.JOY:
                    self.noCmdCount = 0
                    postions = np.array(cmd)*0.005
                    velocity = postions/0.1
                    postions = self.target_positions + postions
                    self.PVTPoint(postions=postions,velocity=velocity,duration=0.1)
                elif cmd.mode == Trajectory.Request.PATH:
                    self.noCmdCount = 0
                    duration = cmd.duration
                    postions = np.array(cmd.positions)
                    velocity = np.array(cmd.velocity)
                    # velocity = (postions-self.target_positions)
                    # for i in range(3,6):
                    #     if abs(velocity[i]) > np.pi:
                    #         velocity[i] = velocity[i] - np.sign(velocity[i])*2*np.pi
                    # velocity = velocity/duration

                    if cmd.idx == 0:
                        duration = 2
                        mv = 0.05
                        mw = 0.5
                        dp = (postions-self.current_positions)
                        for i in range(3,6):
                            if abs(dp[i]) > np.pi:
                                dp[i] = dp[i] - np.sign(dp[i])*2*np.pi
                        t = np.concatenate([dp[:3]/mv, dp[3:]/mw])
                        duration = np.max(np.abs(t))
                        velocity = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

                    print(cmd.idx)
                    print(postions)
                    print(velocity)
                    print(duration)
                    print()

                    for v in velocity[:3]:
                        assert abs(v) < 0.5
                    for r in velocity[3:]:
                        assert abs(r) < 2

                    self.PVTPoint(postions=postions,velocity=velocity,duration=duration)
                    self.gripList.append([time.time(),duration,cmd.grip])
            else:
                self.noCmdCount += 1
                if self.noCmdCount > 1000:
                    self.noCmdCount = 0
                    # self.PVTExit()
                    self.PVTEnter()
    
def main(args=None):
    rclpy.init(args=args)
    arm = Arm()
    rclpy.spin(arm)
    rclpy.shutdown()