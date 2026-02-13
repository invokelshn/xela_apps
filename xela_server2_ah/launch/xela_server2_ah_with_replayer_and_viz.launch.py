from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ah_share = FindPackageShare('xela_server2_ah')
    replayer_share = FindPackageShare('sim_xela_server')
    viz_share = FindPackageShare('xela_taxel_viz_ahv4')

    default_frame_ids = PathJoinSubstitution([ah_share, 'config', 'server2_ah_config.yaml'])
    default_replayer_params = PathJoinSubstitution([replayer_share, 'config', 'replayer_presets.yaml'])
    default_mapping = PathJoinSubstitution([viz_share, 'config', 'taxel_joint_map_new.yaml'])
    default_pattern = PathJoinSubstitution([viz_share, 'config', 'pattern_lahv4.yaml'])
    default_viz_launch = PathJoinSubstitution([viz_share, 'launch', 'xela_taxel_viz_ahv4.launch.py'])
    default_viz_urdf = PathJoinSubstitution([viz_share, 'description', 'xela_uSCuAH_0_modules.xacro'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_name',
            default_value='XR23AHLCPP_left',
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
            'viz_mapping_yaml',
            default_value=default_mapping,
            description='Path to taxel_joint_map_new.yaml for visualization.',
        ),
        DeclareLaunchArgument(
            'viz_pattern_yaml',
            default_value=default_pattern,
            description='Path to pattern_lahv4.yaml for visualization.',
        ),
        DeclareLaunchArgument(
            'viz_launch_file',
            default_value=default_viz_launch,
            description='Launch file for xela_taxel_viz_ahv4.',
        ),
        DeclareLaunchArgument(
            'viz_mode',
            default_value='grid',
            description='Visualization mode: grid or urdf.',
        ),
        DeclareLaunchArgument(
            'overlay_grid_in_urdf',
            default_value='false',
            description='Overlay the grid when viz_mode=urdf.',
        ),
        DeclareLaunchArgument(
            'viz_urdf_xacro_path',
            default_value=default_viz_urdf,
            description='Xacro path used for URDF mode.',
        ),
        DeclareLaunchArgument(
            'viz_sequence',
            default_value='0',
            description='Sequence number for Allegro Hand taxel prefix.',
        ),
        DeclareLaunchArgument(
            'viz_tips',
            default_value='curved',
            description='Fingertip type for Allegro hand URDF.',
        ),
        DeclareLaunchArgument(
            'viz_parent',
            default_value='world',
            description='Parent frame for Allegro hand URDF.',
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
                    package='xela_server2_ah',
                    executable='xela_server2_ah_node',
                    name='xela_server2_ah',
                    output='screen',
                    parameters=[{
                        'ws_host': LaunchConfiguration('ws_host'),
                        'ws_port': LaunchConfiguration('ws_port'),
                        'frame_ids_yaml': LaunchConfiguration('frame_ids_yaml'),
                    }],
                ),
            ],
        ),
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(LaunchConfiguration('viz_launch_file')),
                    launch_arguments={
                        'viz_mode': LaunchConfiguration('viz_mode'),
                        'overlay_grid_in_urdf': LaunchConfiguration('overlay_grid_in_urdf'),
                        'mapping_yaml': LaunchConfiguration('viz_mapping_yaml'),
                        'pattern_yaml': LaunchConfiguration('viz_pattern_yaml'),
                        'urdf_xacro_path': LaunchConfiguration('viz_urdf_xacro_path'),
                        'sequence': LaunchConfiguration('viz_sequence'),
                        'tips': LaunchConfiguration('viz_tips'),
                        'parent': LaunchConfiguration('viz_parent'),
                    }.items(),
                ),
            ],
        ),
    ])
