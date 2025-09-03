
import rclpy.time_source
from tm_msgs.srv import SetPositions,SetEvent
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
        self.gripper_pub = self.create_publisher(GripperCmd, '/gripper/cmd', 10)
        self.pos_sub = self.create_subscription(FeedbackState, 'feedback_states', self.pos_callback, 10)
        while not self.pos_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.event_cli.wait_for_service(timeout_sec=1.0):
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

    def send_request(self,positions=[0.2, -0.4, 0.35, 3.14159, 0.0, -1.57],
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


def main(args=None):
    rclpy.init(args=args)
    armCmd = ArmCmd()
    rclpy.spin_once(armCmd)
    armCmd.send_gripper(0.085)

    # response = armCmd.send_request()
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    # response = armCmd.send_request([0.33, -0.47, 0.35, 3.14159, 0.0, -1.57])    
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    # response = armCmd.send_request([0.33, -0.47, 0.19, 3.14159, 0.0, -1.57])
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)
    
    # armCmd.send_gripper(0.03)
    # time.sleep(1.5)
    # print("pick")
    
    # response = armCmd.send_request([0.33, -0.46, 0.35, 3.14159, 0.0, -1.57])
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    # response = armCmd.send_request([0.2, -0.3, 0.35, 3.14159, 0.0, -1.57])
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    # response = armCmd.send_request([0.2, -0.3, 0.191, 3.14159, 0.0, -1.57])
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    # armCmd.send_gripper(0.085)
    # time.sleep(1.5)
    # print("place")

    # response = armCmd.send_request([0.2, -0.3, 0.35, 3.14159, 0.0, -1.57])
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    # response = armCmd.send_request()
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)
    # ######################################################

    # response = armCmd.send_request([0.2, -0.3, 0.35, 3.14159, 0.0, -1.57])
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    # response = armCmd.send_request([0.2, -0.3, 0.191, 3.14159, 0.0, -1.57])
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    # armCmd.send_gripper(0.03)
    # time.sleep(1.5)
    # print("pick")

    # response = armCmd.send_request([0.2, -0.3, 0.35, 3.14159, 0.0, -1.57])
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    # response = armCmd.send_request([0.33, -0.47, 0.35, 3.14159, 0.0, -1.57])
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    # response = armCmd.send_request([0.33, -0.47, 0.19, 3.14159, 0.0, -1.57])
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)
    
    # armCmd.send_gripper(0.085)
    # time.sleep(1.5)
    # print("place")

    # response = armCmd.send_request([0.33, -0.47, 0.35, 3.14159, 0.0, -1.57])
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    # response = armCmd.send_request()
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    #############################
    # response = armCmd.send_request([0.0, -0.3, 0.35, 3.14159, 0.0, -1.57])
    # while not armCmd.is_arrived():
    #     rclpy.spin_once(armCmd)
    # print("move",armCmd.target_positions)

    # HZ = 100
    # distance = 0.400 #(m)
    # speed = 0.1 #(m/s)
    # total_time = distance/speed

    # duration = 1/(HZ/3)
    # fragment_size = speed*duration
    # p = 0
    # print("total_time %.3f" % total_time,"fragment_size %.3f" % fragment_size ,"duration %.3f" % duration)
    # last = time.time()
    # while True:
        
    #     if p > distance:
    #         break
    #     rclpy.spin_once(armCmd)
    #     if armCmd.is_arrived():
    #         # speed += 0.02
    #         p += fragment_size
    #         armCmd.send_request([p, -0.3, 0.35, 3.14159, 0.0, -1.57],speed,0.001)
    #         print("move",end=" ")
    #         for pos in armCmd.current_positions:
    #             print("%.3f"%pos,end=" ")
    #         print()

    #     while (time.time() - last) < (1/HZ):
    #         rclpy.spin_once(armCmd)
            
    #     print(1/(time.time() - last))
    #     last = time.time()
    while True:
        positions = list(map(float, input("Positions: ").split()))
        if len(positions) == 3:
            positions = positions + [3.14159, 0.0, -1.57]
        if len(positions) == 1:
            positions[0] /= 100.0
            armCmd.send_gripper(positions[0])
            continue
        if len(positions) == 0:
            armCmd.send_event()
        response = armCmd.send_request(positions)
        armCmd.get_logger().info("Response: %s" % response)

    rclpy.spin(armCmd)
    rclpy.shutdown()


if __name__ == '__main__':
    main()