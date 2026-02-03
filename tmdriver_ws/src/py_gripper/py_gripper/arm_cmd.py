
import rclpy.time_source
from tm_msgs.srv import SetPositions,SetEvent,SendScript
from tm_msgs.msg import FeedbackState
from robotiq_85_msgs.msg import GripperCmd

import rclpy
from rclpy.node import Node
import time
# 0.34 -0.47 0.19 target
# move 0.34 -0.47 0.3
# move 0.34 -0.47 0.19
# pick
# move 0.34 -0.47 0.3
# move 0.2 -0.3 0.3
# move 0.2 -0.3 0.19
# place

class ArmCmd(Node):
    def __init__(self):
        super().__init__('arm_cmd')
        self.pos_cli = self.create_client(SetPositions, 'set_positions')
        self.event_cli = self.create_client(SetEvent, 'set_event')
        self.script_cli = self.create_client(SendScript, 'send_script')
        self.gripper_pub = self.create_publisher(GripperCmd, '/gripper/cmd', 10)
        self.pos_sub = self.create_subscription(FeedbackState, 'feedback_states', self.pos_callback, 10)
        while not self.pos_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.event_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.script_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.set_positions_req = SetPositions.Request()
        self.set_event_req = SetEvent.Request()
        self.target_positions = [0.2, -0.4, 0.35, 3.14159, 0.0, -1.57]
        self.current_positions = [0.2, -0.4, 0.35, 3.14159, 0.0, -1.57]
        
    def pos_callback(self,msg):
        self.current_positions = msg.tool_pose
        # self.get_logger().info("Current Position: %s" % self.current_positions)

    def is_arrived(self,error=0.01):
        if sum((self.target_positions[i]-self.current_positions[i])**2 for i in range(3)) > error**2:
            return False
        return True

    def set_positions(self,positions=[0.2, -0.4, 0.35, 3.14159, 0.0, -1.57],
                     velocity=0.1, acc_time=0.5, blend_percentage=100, fine_goal=False):
        
        self.target_positions = positions
        print(self.target_positions)
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
        print(gap)
        grip_msg = GripperCmd()
        grip_msg.emergency_release = False
        grip_msg.emergency_release_dir = 0
        grip_msg.stop = False
        grip_msg.position = gap
        grip_msg.speed = 0.1
        grip_msg.force = 1.0
        self.gripper_pub.publish(grip_msg)

    def send_event(self):
        rclpy.spin_once(self)
        self.set_event_req = SetEvent.Request()
        self.set_event_req.func = SetEvent.Request.STOP
        self.set_event_req.arg0 = 0
        self.set_event_req.arg1 = 0
        future = self.event_cli.call_async(self.set_event_req)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()
    def send_script(self,cmd):
        script_req =  SendScript.Request()
        script_req.id = "arm"
        script_req.script = cmd
        future = self.script_cli.call_async(script_req)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def set_positions_block(self,positions=[0.2, -0.4, 0.35, 3.14159, 0.0, -1.57],
                     velocity=0.2, acc_time=0.25, blend_percentage=100, fine_goal=False):
        response = self.set_positions(positions,velocity,acc_time,blend_percentage,fine_goal)
        while not self.is_arrived():
            rclpy.spin_once(self)
        print("move",self.target_positions)
    

def main(args=None):
    rclpy.init(args=args)
    armCmd = ArmCmd()
    # armCmd.send_script("Env.RunningSpeed = 100")
    armCmd.set_positions_block([0.4,0.0,0.5,3.14,0.0,-1.57])
    # armCmd.send_script('''
    #                    Compliance cp1
    #                    cp1.Frame(1)
    #                    cp1.Multiple("X", true, 200, -200)
    #                    cp1.Multiple("Y", true, 200, -200)
    #                    cp1.Multiple("Z", true, 200, -200)
    #                    int re = cp1.Start()
    #                    ''')
    armCmd.send_script('Compliance cp1')
    armCmd.send_script('cp1.Reset()')
    # armCmd.send_script('cp1.Timeout(10000)')
    armCmd.send_script('cp1.Frame(1)')
    # armCmd.send_script('cp1.Impedance("All", 2)')
    armCmd.send_script('cp1.HighResistance(true)')
    armCmd.send_script('cp1.Multiple("X", true, 200, -200)')
    armCmd.send_script('cp1.Multiple("Y", true, 200, -200)')
    armCmd.send_script('cp1.Multiple("Z", true, 200, -200)')
    armCmd.send_script('cp1.Multiple("RX", true, 270, -270)')
    armCmd.send_script('cp1.Multiple("RY", true, 270, -270)')
    armCmd.send_script('cp1.Multiple("RZ", true, 270, -270)')
    armCmd.send_script('cp1.Start()')

    # armCmd.send_script("cp1.Frame(1)")
    # armCmd.send_script("cp1.HighResistance(true)")
    # #cp1.Single("Z", 40)
    # armCmd.send_script("cp1.Single(\"Z\", 40)")
    # # p1.Teach("Linear", "P1", "P2", 0)// 設置教導點 P1 與 P2 (將改用 Teach 模式)
    # armCmd.send_script("cp1.Teach(\"Linear\", \"P1\", \"P2\", 0)")
    # # cp1.Multiple("X", true, 100, -100)// 設置多軸參數 X 方向 (將改用 Multiple 模式)
    # armCmd.send_script("cp1.Multiple(\"X\", true, 100, -100)")
    # # cp1.Multiple("Z", true, 100, -100)// 設置多軸參數 Z 方向
    # armCmd.send_script("cp1.Multiple(\"Z\", true, 100, -100)")
    # # cp1.Multiple("X", true, 10, -10)// 設置多軸參數 X 方向 (將覆寫前次的 Multiple X 參數)
    # armCmd.send_script("cp1.Multiple(\"X\", true, 10, -10)")
    # # cp1.Impedance("All", 1)// 設置阻抗參數 (將改用 Impedance 模式)
    # armCmd.send_script("cp1.Impedance(\"All\", 1)")
    # # // 停止條件
    # # cp1.Timeout(10000)// 逾時 10000ms
    # armCmd.send_script("cp1.Timeout(10000)")
    # # cp1.DInput("ControlBox", 0, "H")// 當 ControlBox DI0 High 時
    # armCmd.send_script("cp1.DInput(\"ControlBox\", 0, \"H\")")
    # # cp1.AInput("ControlBox", 0, ">=", 3.3)// 當 ControlBox AI0 >= 3.3V 時
    # armCmd.send_script("cp1.AInput(\"ControlBox\", 0, \">=\", 3.3)")
    # # int count = 0
    # armCmd.send_script("int count = 0")
    # # cp1.Condition(count > 1000)// 條件式判斷
    # armCmd.send_script("cp1.Condition(count > 1000)")
    # # cp1.Reset()
    # armCmd.send_script("cp1.Reset()")

    rclpy.spin(armCmd)
    rclpy.shutdown()


if __name__ == '__main__':
    main()