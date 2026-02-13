from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('xela_server2_2f')
    replayer_share = FindPackageShare('sim_xela_server')
    default_frame_ids = PathJoinSubstitution([pkg_share, 'config', 'server2_2f_config.yaml'])
    default_params = PathJoinSubstitution([pkg_share, 'config', 'xela_server2_2f_params.yaml'])
    default_replayer_params = PathJoinSubstitution([replayer_share, 'config', 'replayer_presets.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_name',
            default_value='uSPa35',
            description='Reference JSON name (without .json) under sim_xela_server/resource/',
        ),
        DeclareLaunchArgument(
            'bind_host',
            default_value='0.0.0.0',
            description='WebSocket server bind host.',
        ),
        DeclareLaunchArgument(
            'bind_port',
            default_value='5000',
            description='WebSocket server bind port.',
        ),
        DeclareLaunchArgument(
            'ws_host',
            default_value='localhost',
            description='WebSocket client host (xela_server2_2f_node).',
        ),
        DeclareLaunchArgument(
            'ws_port',
            default_value='5000',
            description='WebSocket client port (xela_server2_2f_node).',
        ),
        DeclareLaunchArgument(
            'frame_ids_yaml',
            default_value=default_frame_ids,
            description='Path to server2_2f_config.yaml',
        ),
        DeclareLaunchArgument(
            'warmup_sec',
            default_value='5.0',
            description='Replayer warmup time before applying sine variation.',
        ),
        DeclareLaunchArgument(
            'preset',
            default_value='normal',
            description='Replayer speed preset: slow | normal | fast | custom.',
        ),
        DeclareLaunchArgument(
            'publish_period_ms',
            default_value='200',
            description='Replayer publish period (ms).',
        ),
        DeclareLaunchArgument(
            'sine_freq_hz',
            default_value='0.05',
            description='Replayer sine frequency (Hz).',
        ),
        DeclareLaunchArgument(
            'replayer_params_file',
            default_value=default_replayer_params,
            description='Path to replayer preset parameters file.',
        ),
        Node(
            package='sim_xela_server',
            executable='sim_xela_server_node',
            name='websocket_replayer_server',
            output='screen',
            parameters=[
                LaunchConfiguration('replayer_params_file'),
                {
                    'model_name': LaunchConfiguration('model_name'),
                    'bind_host': LaunchConfiguration('bind_host'),
                    'bind_port': LaunchConfiguration('bind_port'),
                    'warmup_sec': LaunchConfiguration('warmup_sec'),
                    'preset': LaunchConfiguration('preset'),
                    'publish_period_ms': LaunchConfiguration('publish_period_ms'),
                    'sine_freq_hz': LaunchConfiguration('sine_freq_hz'),
                },
            ],
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='xela_server2_2f',
                    executable='xela_server2_2f_node',
                    name='xela_server2_2f',
                    output='screen',
                    parameters=[
                        default_params,
                        {
                            'ws_host': LaunchConfiguration('ws_host'),
                            'ws_port': LaunchConfiguration('ws_port'),
                            'frame_ids_yaml': LaunchConfiguration('frame_ids_yaml'),
                        },
                    ],
                ),
            ],
        ),
    ])
