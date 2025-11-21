from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
import launch_ros.actions

#def launch(launch_descriptor, argv):
def generate_launch_description():


    akmcar = LaunchConfiguration('akmcar', default='true')
    usart_port_name = LaunchConfiguration('usart_port_name', default='/dev/wheeltec_controller')
    serial_baud_rate = LaunchConfiguration('serial_baud_rate', default='115200')
    return LaunchDescription([
        DeclareLaunchArgument('usart_port_name', default_value='/dev/wheeltec_controller'),
        DeclareLaunchArgument('serial_baud_rate', default_value='115200'),
        launch_ros.actions.Node(
            package='turn_on_wheeltec_robot',
            executable='wheeltec_robot_node',
            output='screen',
            parameters=[{'usart_port_name': usart_port_name,
                'serial_baud_rate': serial_baud_rate,
                'robot_frame_id': 'base_footprint',
                'odom_frame_id': 'odom_combined',
                'cmd_vel': 'cmd_vel',
                'akm_cmd_vel': 'none',
                'product_number': 0,
                'odom_x_scale': 1.0,
                'odom_y_scale': 1.0,
                'odom_z_scale_positive': 1.0,
                'odom_z_scale_negative': 1.0}],
        )
    ])
