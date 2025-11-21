import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    usart_port_name = LaunchConfiguration('usart_port_name', default='/dev/wheeltec_controller')
    serial_baud_rate = LaunchConfiguration('serial_baud_rate', default='115200')

    turn_pkg = get_package_share_directory('turn_on_wheeltec_robot')
    turn_launch = os.path.join(turn_pkg, 'launch')
    nav_pkg = get_package_share_directory('wheeltec_nav2')
    nav_launch = os.path.join(nav_pkg, 'launch')
    nav_map = os.path.join(nav_pkg, 'map', 'WHEELTEC.yaml')
    nav_params = os.path.join(nav_pkg, 'param', 'wheeltec_params', 'param_mini_mec.yaml')

    astra_pkg = get_package_share_directory('astra_camera')
    astra_launch = os.path.join(astra_pkg, 'launch', 'astra_pro.launch.xml')

    map_file = LaunchConfiguration('map', default=nav_map)
    param_file = LaunchConfiguration('params', default=nav_params)

    aruco_marker_id = LaunchConfiguration('marker_id', default='582')
    aruco_marker_size = LaunchConfiguration('marker_size', default='0.1')
    ee_image = LaunchConfiguration('ee_image', default='/ee_cam/color/image_raw')
    ee_camera_info = LaunchConfiguration('ee_camera_info', default='/ee_cam/color/camera_info')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('map', default_value=map_file))
    ld.add_action(DeclareLaunchArgument('params', default_value=param_file))
    ld.add_action(DeclareLaunchArgument('usart_port_name', default_value=usart_port_name))
    ld.add_action(DeclareLaunchArgument('serial_baud_rate', default_value=serial_baud_rate))
    ld.add_action(DeclareLaunchArgument('marker_id', default_value=aruco_marker_id))
    ld.add_action(DeclareLaunchArgument('marker_size', default_value=aruco_marker_size))
    ld.add_action(DeclareLaunchArgument('ee_image', default_value=ee_image))
    ld.add_action(DeclareLaunchArgument('ee_camera_info', default_value=ee_camera_info))

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([turn_launch, '/turn_on_wheeltec_robot.launch.py']),
                    launch_arguments={'usart_port_name': usart_port_name,
                                      'serial_baud_rate': serial_baud_rate}.items()))
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([turn_launch, '/wheeltec_lidar.launch.py'])))
    ld.add_action(IncludeLaunchDescription(XMLLaunchDescriptionSource(astra_launch)))
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([nav_launch, '/bringup_launch.py']),
                    launch_arguments={'map': map_file,
                                      'use_sim_time': use_sim_time,
                                      'params_file': param_file}.items()))
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource([get_package_share_directory('rdkx5_net_bridge'), '/launch/net_bridge.launch.py'])))

    # ESP32 外挂桥接已取消，RDKX5 原生 Wi-Fi/BT 使用上位机通信，不再引入串口桥接

    aruco = Node(package='aruco_ros', executable='single', parameters=[{
        'image_is_rectified': True,
        'marker_size': aruco_marker_size,
        'marker_id': aruco_marker_id,
        'reference_frame': '',
        'camera_frame': 'ee_cam_optical',
        'marker_frame': 'aruco_marker'}],
        remappings=[('/camera_info', ee_camera_info),('/image', ee_image)])
    ld.add_action(aruco)

    grasp_planner = Node(package='wheeltec_grasp_mvp', executable='grasp_planner', parameters=[{
        'marker_pose_topic': '/aruco_single/pose',
        'output_frame': 'base_link',
        'approach_offset': 0.10,
        'output_topic': '/grasp_pose'}])
    ld.add_action(grasp_planner)

    return ld