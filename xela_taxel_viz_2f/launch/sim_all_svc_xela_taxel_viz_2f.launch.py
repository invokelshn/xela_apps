from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    viz_share = FindPackageShare('xela_taxel_viz_2f')
    server_share = FindPackageShare('xela_server2_2f')
    grid_rviz = PathJoinSubstitution([viz_share, 'config', 'grid_xela_taxel_viz_2f.rviz'])
    urdf_rviz = PathJoinSubstitution([viz_share, 'config', 'urdf_xela_taxel_viz_2f.rviz'])
    server_launch = PathJoinSubstitution([server_share, 'launch', 'xela_server2_2f_with_replayer.launch.py'])
    default_frame_ids = PathJoinSubstitution([server_share, 'config', 'server2_2f_config.yaml'])
    replayer_share = FindPackageShare('sim_xela_server')
    default_replayer_params = PathJoinSubstitution([replayer_share, 'config', 'replayer_presets.yaml'])
    viz_launch = PathJoinSubstitution([viz_share, 'launch', 'xela_taxel_viz_2f.launch.py'])

    def build_rviz_node(context):
        viz_mode = LaunchConfiguration('viz_mode').perform(context)
        cfg_override = LaunchConfiguration('rviz_config').perform(context).strip()
        cfg = cfg_override or (grid_rviz if viz_mode == 'grid' else urdf_rviz)
        return [
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', cfg],
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('model_name', default_value='uSPr2F'),
        DeclareLaunchArgument('viz_mode', default_value='grid'),
        DeclareLaunchArgument('style_preset', default_value='default'),
        DeclareLaunchArgument('overlay_grid_in_urdf', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=''),

        DeclareLaunchArgument('model_name', default_value='uSPa35'),
        DeclareLaunchArgument('bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('bind_port', default_value='5000'),
        DeclareLaunchArgument('ws_host', default_value='localhost'),
        DeclareLaunchArgument('ws_port', default_value='5000'),
        DeclareLaunchArgument('frame_ids_yaml', default_value=default_frame_ids),
        DeclareLaunchArgument('warmup_sec', default_value='5.0'),
        DeclareLaunchArgument('preset', default_value='normal'),
        DeclareLaunchArgument('publish_period_ms', default_value='200'),
        DeclareLaunchArgument('sine_freq_hz', default_value='0.05'),
        DeclareLaunchArgument('replayer_params_file', default_value=default_replayer_params),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(server_launch),
            launch_arguments={
                'model_name': LaunchConfiguration('model_name'),
                'bind_host': LaunchConfiguration('bind_host'),
                'bind_port': LaunchConfiguration('bind_port'),
                'ws_host': LaunchConfiguration('ws_host'),
                'ws_port': LaunchConfiguration('ws_port'),
                'frame_ids_yaml': LaunchConfiguration('frame_ids_yaml'),
                'warmup_sec': LaunchConfiguration('warmup_sec'),
                'preset': LaunchConfiguration('preset'),
                'publish_period_ms': LaunchConfiguration('publish_period_ms'),
                'sine_freq_hz': LaunchConfiguration('sine_freq_hz'),
                'replayer_params_file': LaunchConfiguration('replayer_params_file'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(viz_launch),
            launch_arguments={
                'model_name': LaunchConfiguration('model_name'),
                'viz_mode': LaunchConfiguration('viz_mode'),
                'style_preset': LaunchConfiguration('style_preset'),
                'overlay_grid_in_urdf': LaunchConfiguration('overlay_grid_in_urdf'),
            }.items(),
        ),

        OpaqueFunction(function=build_rviz_node),
    ])
