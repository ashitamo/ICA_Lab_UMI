# Example of publishing a Path message in a ROS2 Python node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, '/robot_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map" # Or your robot's base frame
        self.posx = 0.0

    def publish_path(self):
        # Add a new pose to the path (example: a simple straight line)
        pose = PoseStamped()
        pose.header.frame_id = self.path_msg.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.posx
        self.posx += 0.1
        pose.pose.orientation.w = 1.0 # Identity quaternion
        self.path_msg.poses.append(pose)

        self.publisher_.publish(self.path_msg)
        self.get_logger().info(f'Publishing path with {len(self.path_msg.poses)} poses')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()