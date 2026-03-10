from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            SetLaunchConfiguration, TimerAction)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ah_share = FindPackageShare('xela_server2_ah')
    replayer_share = FindPackageShare('xela_atag_mock_server')
    default_frame_ids = PathJoinSubstitution([ah_share, 'config', 'server2_ah_config.yaml'])
    default_replayer_params = PathJoinSubstitution([replayer_share, 'config', 'replayer_presets.yaml'])

    def select_hand_side(context):
        model_name = LaunchConfiguration('model_name').perform(context).strip()
        hand_side_override = LaunchConfiguration('hand_side').perform(context).strip()
        hand_side = hand_side_override or (
            'right' if model_name in ('XR23AHRCPP', 'XR23AHRCPP_right') else 'left'
        )
        return [
            SetLaunchConfiguration('hand_side', hand_side),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_name',
            default_value='XR23AHLCPP',
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
            description='WebSocket client host (xela_server2_ah_node).',
        ),
        DeclareLaunchArgument(
            'ws_port',
            default_value='5000',
            description='WebSocket client port (xela_server2_ah_node).',
        ),
        DeclareLaunchArgument(
            'frame_ids_yaml',
            default_value=default_frame_ids,
            description='Path to server2_ah_config.yaml',
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
        DeclareLaunchArgument(
            'integer_z_range',
            default_value='10085.0',
            description='Simulated integer Z range (raw taxel units).',
        ),
        DeclareLaunchArgument(
            'calib_z_range',
            default_value='14.0',
            description='Simulated calibrated Z range.',
        ),
        DeclareLaunchArgument(
            'z_bias_power',
            default_value='1.0',
            description='Z bias power (>1.0 favors smaller values).',
        ),
        DeclareLaunchArgument(
            'hand_side',
            default_value='',
            description='Hand side for frame_id prefix: left or right.',
        ),
        OpaqueFunction(function=select_hand_side),
        Node(
            package='xela_atag_mock_server',
            executable='xela_atag_mock_server_node',
            name='xela_atag_mock_server',
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
                    'integer_z_range': LaunchConfiguration('integer_z_range'),
                    'calib_z_range': LaunchConfiguration('calib_z_range'),
                    'z_bias_power': LaunchConfiguration('z_bias_power'),
                },
            ],
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='xela_server2_ah',
                    executable='xela_server2_ah_node',
                    name='xela_server2_ah',
                    output='screen',
                    parameters=[{
                        'ws_host': LaunchConfiguration('ws_host'),
                        'ws_port': LaunchConfiguration('ws_port'),
                        'frame_ids_yaml': LaunchConfiguration('frame_ids_yaml'),
                        'hand_side': LaunchConfiguration('hand_side'),
                    }],
                ),
            ],
        ),
    ])
