from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare('sim_xela_server')
    default_replayer_params = PathJoinSubstitution([pkg_share, 'config', 'replayer_presets.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_name',
            default_value='XR23AHLCPP',
            description='Reference JSON name (without .json) under resource/',
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
            'warmup_sec',
            default_value='5.0',
            description='Replayer warmup time before applying sine variation.',
        ),
        DeclareLaunchArgument(
            'preset',
            default_value='H',
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
            'replayer_params_file',
            default_value=default_replayer_params,
            description='Path to replayer preset parameters file.',
        ),
        Node(
            package='sim_xela_server',
            executable='sim_xela_server_node',
            name='sim_xela_server',
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
    ])
