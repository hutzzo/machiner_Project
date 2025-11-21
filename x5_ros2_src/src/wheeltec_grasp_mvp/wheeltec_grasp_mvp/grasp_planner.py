import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf_transformations as tft
import tf2_ros

class GraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner')
        self.declare_parameter('marker_pose_topic', '/aruco_single/pose')
        self.declare_parameter('output_topic', '/grasp_pose')
        self.declare_parameter('approach_offset', 0.10)
        self.declare_parameter('output_frame', 'base_link')
        self.marker_topic = self.get_parameter('marker_pose_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.offset = self.get_parameter('approach_offset').get_parameter_value().double_value
        self.output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.sub = self.create_subscription(PoseStamped, self.marker_topic, self.cb, 10)

    def cb(self, msg: PoseStamped):
        p = msg.pose
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        R = tft.quaternion_matrix(q)
        t = [p.position.x, p.position.y, p.position.z]
        T = tft.translation_matrix(t)
        M = T @ R
        dz = [0, 0, self.offset, 1.0]
        target = M @ dz
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.pose.position.x = target[0]
        out.pose.position.y = target[1]
        out.pose.position.z = target[2]
        out.pose.orientation = msg.pose.orientation
        if self.output_frame and self.output_frame != out.header.frame_id:
            try:
                tr = self.buffer.lookup_transform(self.output_frame, out.header.frame_id, rclpy.time.Time())
                import tf2_geometry_msgs
                out = tf2_geometry_msgs.do_transform_pose(out, tr)
                out.header.frame_id = self.output_frame
            except Exception:
                pass
        self.pub.publish(out)

def main():
    rclpy.init()
    node = GraspPlanner()
    rclpy.spin(node)
    rclpy.shutdown()