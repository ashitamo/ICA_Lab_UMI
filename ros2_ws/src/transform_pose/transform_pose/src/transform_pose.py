import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu,Image
from scipy.spatial.transform import Rotation
from collections import deque

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        # Subscribe to a topic
        
        self.buffer_size = deque(maxlen=20)
        self.data_list = []
        self.create_subscription(Image,'/camera/infra1/image_rect_raw',self.infra1_callback,10)
        self.create_subscription(Imu,'/poseimu',self.poseimu_callback,10)
        
    def poseimu_callback(self, msg):
        self.buffer_size.append(msg)

    def infra1_callback(self, msg):
        target_stamp = msg.header.stamp
        imu_pose_new = self.interpolate_imu(target_stamp)
        if imu_pose_new:
            timestamp = target_stamp.sec + target_stamp.nanosec * 1e-9
            pose = imu_pose_new
            self.data_list.append([timestamp,pose,msg])  
        
    def interpolate_imu(self, target_stamp):
        target_time = target_stamp.sec + target_stamp.nanosec * 1e-9
        msg_b4 = None
        msg_after = None
        for msg in self.buffer_size:
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg_time < target_time:
                msg_b4 = msg
            elif msg_time > target_time and msg_after is None:
                msg_after = msg
                break
        if msg_b4 and msg_after:
            return self.interpolate_pos_orient(msg_b4, msg_after, target_stamp)
        return None

            
    def interpolate_pos_orient(self,msg_b4, msg_after, target_stamp):
        t1 = msg_b4.header.stamp.sec + msg_b4.header.stamp.nanosec * 1e-9
        t2 = msg_after.header.stamp.sec + msg_after.header.stamp.nanosec * 1e-9
        t = target_stamp.sec + target_stamp.nanosec * 1e-9
        if t1 == t:
            quat = [msg_b4.pose.pose.orientation.x, msg_b4.pose.pose.orientation.y, msg_b4.pose.pose.orientation.z, msg_b4.pose.pose.orientation.w]
            return msg_b4.pose.pose.position.x, msg_b4.pose.pose.position.y, msg_b4.pose.pose.position.z, quat
        elif t2 == t:
            quat = [msg_after.pose.pose.orientation.x, msg_after.pose.pose.orientation.y, msg_after.pose.pose.orientation.z, msg_after.pose.pose.orientation.w]
            return msg_after.pose.pose.position.x, msg_after.pose.pose.position.y, msg_after.pose.pose.position.z, quat
        if t2 == t1:
        # Avoid division by zero, just return msg_b4
            quat = [msg_b4.pose.pose.orientation.x, msg_b4.pose.pose.orientation.y, msg_b4.pose.pose.orientation.z, msg_b4.pose.pose.orientation.w]
            return msg_b4.pose.pose.position.x, msg_b4.pose.pose.position.y, msg_b4.pose.pose.position.z, quat
        x1, y1, z1 = msg_b4.pose.pose.position.x, msg_b4.pose.pose.position.y, msg_b4.pose.pose.position.z
        x2, y2, z2 = msg_after.pose.pose.position.x, msg_after.pose.pose.position.y, msg_after.pose.pose.position.z
        
        ratio = (t - t1) / (t2 - t1)
        x_interp = x1 + ratio * (x2 - x1)
        y_interp = y1 + ratio * (y2 - y1)
        z_interp = z1 + ratio * (z2 - z1)
    
        q1 = [msg_b4.pose.pose.orientation.x, msg_b4.pose.pose.orientation.y,msg_b4.pose.pose.orientation.z, msg_b4.pose.pose.orientation.w]
        q2 = [msg_after.pose.pose.orientation.x, msg_after.pose.pose.orientation.y,msg_after.pose.pose.orientation.z, msg_after.pose.pose.orientation.w]
        rot1 = Rotation.from_quat(q1)
        rot2 = Rotation.from_quat(q2)
        slerped_rot = Rotation.slerp(rot1, rot2, [0, 1])([ratio])[0]
        q_interp = slerped_rot.as_quat()

        return x_interp, y_interp, z_interp, q_interp
    
            
def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()