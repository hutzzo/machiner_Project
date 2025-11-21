from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    net_cmd_vel = LaunchConfiguration('net_cmd_vel_topic', default='/net/cmd_vel')
    net_moveit = LaunchConfiguration('net_moveit_joint_topic', default='/net/moveit_joint_cmd')
    net_pose = LaunchConfiguration('net_ee_pose_topic', default='/net/ee_pose_target')
    f_cmd_vel = LaunchConfiguration('forward_cmd_vel_topic', default='cmd_vel')
    f_moveit = LaunchConfiguration('forward_moveit_joint_topic', default='stm32_moveit_joint_cmd')
    f_pose = LaunchConfiguration('forward_ee_pose_topic', default='/ee_pose_target')
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('net_cmd_vel_topic', default_value=net_cmd_vel))
    ld.add_action(DeclareLaunchArgument('net_moveit_joint_topic', default_value=net_moveit))
    ld.add_action(DeclareLaunchArgument('net_ee_pose_topic', default_value=net_pose))
    ld.add_action(DeclareLaunchArgument('forward_cmd_vel_topic', default_value=f_cmd_vel))
    ld.add_action(DeclareLaunchArgument('forward_moveit_joint_topic', default_value=f_moveit))
    ld.add_action(DeclareLaunchArgument('forward_ee_pose_topic', default_value=f_pose))
    node = Node(package='rdkx5_net_bridge', executable='net_bridge', parameters=[{
        'net_cmd_vel_topic': net_cmd_vel,
        'net_moveit_joint_topic': net_moveit,
        'net_ee_pose_topic': net_pose,
        'forward_cmd_vel_topic': f_cmd_vel,
        'forward_moveit_joint_topic': f_moveit,
        'forward_ee_pose_topic': f_pose
    }])
    ld.add_action(node)
    return ld