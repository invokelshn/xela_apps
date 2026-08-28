from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ws_host', default_value='localhost'),
        DeclareLaunchArgument('ws_port', default_value='5000'),
        DeclareLaunchArgument('hand_side', default_value='right',
                              description='Hand side: left or right'),
        DeclareLaunchArgument('header_frame_id', default_value=''),
        DeclareLaunchArgument('use_ros_time_for_sensor_time', default_value='false'),
        DeclareLaunchArgument('publisher_qos_depth', default_value='10'),
        DeclareLaunchArgument('input_json_path', default_value=''),
        DeclareLaunchArgument('playback_interval_ms', default_value='100'),
        DeclareLaunchArgument('playback_loop', default_value='true'),
        Node(
            package='xela_server2_dg5f',
            executable='xela_server2_dg5f_node',
            name='xela_server2_dg5f',
            parameters=[{
                'ws_host': LaunchConfiguration('ws_host'),
                'ws_port': LaunchConfiguration('ws_port'),
                'hand_side': LaunchConfiguration('hand_side'),
                'header_frame_id': LaunchConfiguration('header_frame_id'),
                'use_ros_time_for_sensor_time': LaunchConfiguration('use_ros_time_for_sensor_time'),
                'publisher_qos_depth': LaunchConfiguration('publisher_qos_depth'),
                'input_json_path': LaunchConfiguration('input_json_path'),
                'playback_interval_ms': LaunchConfiguration('playback_interval_ms'),
                'playback_loop': LaunchConfiguration('playback_loop'),
            }],
            output='screen',
        ),
    ])
