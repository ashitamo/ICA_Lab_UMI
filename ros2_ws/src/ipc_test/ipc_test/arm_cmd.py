import rclpy.time_source
from tm_msgs.srv import SetPositions,SetEvent
from tm_msgs.msg import FeedbackState
from robotiq_85_msgs.msg import GripperCmd
import rclpy
from rclpy.node import Node
import time
from icp_node import *
import numpy as np
import yaml

with open('/home/hsun91/ICA_Lab_UMI/ICA_Lab_UMI_Config.yaml', 'r') as file:
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


def main(args=None):
    rclpy.init(args=args)
    iCPNode = startNode()
    armCmd = ArmCmd()
    rclpy.spin_once(armCmd)
    armCmd.send_gripper(0.085)

    # init position
    response = armCmd.set_positions()
    while not armCmd.is_arrived():
        rclpy.spin_once(armCmd)
    print("move",armCmd.target_positions)

    # approach 
    #   move to top position
    response = armCmd.set_positions([0.33, -0.47, 0.35, 3.14159, 0.0, -1.57])
    while not armCmd.is_arrived():
        rclpy.spin_once(armCmd)
    print("move",armCmd.target_positions)

    #   move to pick position
    response = armCmd.set_positions([0.33, -0.47, 0.35, 3.14159, 0.0, -1.57])
    while not armCmd.is_arrived():
        rclpy.spin_once(armCmd)
    print("move",armCmd.target_positions)

    #   ICP
    data = iCPNode.get_cam_data()
    intr = iCPNode.get_cam_info()
    color_img, depth_img, point_cloud = data
    field_img_mask_center, field_img_colored_mask, field_masks = generate_masks(color_img)
    mask,_ = get_cylinder_mask(
            [],
            field_img_mask_center,
            field_masks
        )
    src = peg_tgt
    tgt = mesh_to_pcd("peg_30.5mm - Part 1.stl")
    T, bbox, result_icp, _, _ = get_bbox_by_icp(src, tgt, viz=True, all_pcd=point_cloud)
    
    #   cal grasp position
    # top_pos, top_R, bottom_pos, bottom_R = top_and_bottom_face_centers(bbox)
    # def c2e(p_C0_o0,T_a_C0):
    #     '''
    #     p_C0_o0 : np.array([x,y,z])
    #     T_a_C0 : np.array([x,y,z,rx,ry,rz,rw])
    #     =>
    #     T_a_C1 np.array([x,y,z,rx,ry,rz,rw])
    #     '''

    #     T_C0_o0 = np.array(
    #         [[ 1, 0, 0, p_C0_o0[0]],
    #         [ 0, 1, 0, p_C0_o0[1]],
    #         [ 0, 0, 1, p_C0_o0[2]],
    #         [ 0, 0, 0, 1]]
    #     )
    #     temp = R.from_quat(T_a_C0[3:]).as_matrix()
    #     T_a_C0 = np.array(
    #         [[ temp[0,0], temp[0,1], temp[0,2], T_a_C0[0]],
    #         [ temp[1,0], temp[1,1], temp[1,2], T_a_C0[1]],
    #         [ temp[2,0], temp[2,1], temp[2,2], T_a_C0[2]],
    #         [ 0, 0, 0, 1]]
    #     )
    #     T_a_o0 = T_a_C0 @ T_C0_o0
    #     T_a_E1 = T_a_o0
    #     rot = R.from_matrix(T_a_E1[:3,:3]).as_euler('xyz', degrees=True)
    #     rot[0],rot[1] = 180.0,0.0
    #     T_a_E1[:3,:3] = R.from_euler('xyz', rot, degrees=True).as_matrix()
    #     T_a_C1 = T_a_E1 @ T_E_C
    #     temp = R.from_matrix(T_a_C1[:3,:3]).as_quat()
    #     T_a_C1 = np.array([T_a_C1[0,3],T_a_C1[1,3],T_a_C1[2,3],temp[0],temp[1],temp[2],temp[3]])
    #     return T_a_C1
    
    # c2e(top_pos, top_R)




    rclpy.spin(armCmd)
    rclpy.shutdown()


if __name__ == '__main__':
    main()