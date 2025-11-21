from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    marker_id = LaunchConfiguration('marker_id', default='582')
    marker_size = LaunchConfiguration('marker_size', default='0.1')
    ee_image = LaunchConfiguration('ee_image', default='/ee_cam/color/image_raw')
    ee_camera_info = LaunchConfiguration('ee_camera_info', default='/ee_cam/color/camera_info')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('marker_id', default_value=marker_id))
    ld.add_action(DeclareLaunchArgument('marker_size', default_value=marker_size))
    ld.add_action(DeclareLaunchArgument('ee_image', default_value=ee_image))
    ld.add_action(DeclareLaunchArgument('ee_camera_info', default_value=ee_camera_info))

    aruco = Node(package='aruco_ros', executable='single', parameters=[{
        'image_is_rectified': True,
        'marker_size': marker_size,
        'marker_id': marker_id,
        'reference_frame': '',
        'camera_frame': 'ee_cam_optical',
        'marker_frame': 'aruco_marker'}],
        remappings=[('/camera_info', ee_camera_info),('/image', ee_image)])
    ld.add_action(aruco)

    grasp = Node(package='wheeltec_grasp_mvp', executable='grasp_planner', parameters=[{
        'marker_pose_topic': '/aruco_single/pose',
        'output_frame': 'base_link',
        'approach_offset': 0.10,
        'output_topic': '/grasp_pose'}])
    ld.add_action(grasp)

    return ld