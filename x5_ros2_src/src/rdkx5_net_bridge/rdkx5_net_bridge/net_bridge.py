import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray

class NetBridge(Node):
    def __init__(self):
        super().__init__('net_bridge')
        self.declare_parameter('net_cmd_vel_topic', '/net/cmd_vel')
        self.declare_parameter('net_moveit_joint_topic', '/net/moveit_joint_cmd')
        self.declare_parameter('net_ee_pose_topic', '/net/ee_pose_target')
        self.declare_parameter('forward_cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('forward_moveit_joint_topic', 'stm32_moveit_joint_cmd')
        self.declare_parameter('forward_ee_pose_topic', '/ee_pose_target')
        self.pub_cmd_vel = self.create_publisher(Twist, self.get_parameter('forward_cmd_vel_topic').get_parameter_value().string_value, 10)
        self.pub_moveit = self.create_publisher(Float32MultiArray, self.get_parameter('forward_moveit_joint_topic').get_parameter_value().string_value, 10)
        self.pub_pose = self.create_publisher(PoseStamped, self.get_parameter('forward_ee_pose_topic').get_parameter_value().string_value, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, self.get_parameter('net_cmd_vel_topic').get_parameter_value().string_value, self.cb_cmd_vel, 10)
        self.sub_moveit = self.create_subscription(Float32MultiArray, self.get_parameter('net_moveit_joint_topic').get_parameter_value().string_value, self.cb_moveit, 10)
        self.sub_pose = self.create_subscription(PoseStamped, self.get_parameter('net_ee_pose_topic').get_parameter_value().string_value, self.cb_pose, 10)
    def cb_cmd_vel(self, msg: Twist):
        self.pub_cmd_vel.publish(msg)
    def cb_moveit(self, msg: Float32MultiArray):
        self.pub_moveit.publish(msg)
    def cb_pose(self, msg: PoseStamped):
        self.pub_pose.publish(msg)

def main():
    rclpy.init()
    node = NetBridge()
    rclpy.spin(node)
    rclpy.shutdown()