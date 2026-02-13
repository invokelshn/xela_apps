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
    server_launch = PathJoinSubstitution([server_share, 'launch', 'xela_server2_2f_with_server.launch.py'])
    default_frame_ids = PathJoinSubstitution([server_share, 'config', 'server2_2f_config.yaml'])
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
        DeclareLaunchArgument('viz_mode', default_value='urdf'),
        DeclareLaunchArgument('style_preset', default_value='default'),
        DeclareLaunchArgument('overlay_grid_in_urdf', default_value='false'),
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument('ws_host', default_value='localhost'),
        DeclareLaunchArgument('ws_port', default_value='5000'),
        DeclareLaunchArgument('frame_ids_yaml', default_value=default_frame_ids),
        DeclareLaunchArgument('xela_server_exec', default_value='/etc/xela/xela_server'),
        DeclareLaunchArgument('rviz_config', default_value=''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(server_launch),
            launch_arguments={
                'can_port': LaunchConfiguration('can_port'),
                'ws_host': LaunchConfiguration('ws_host'),
                'ws_port': LaunchConfiguration('ws_port'),
                'frame_ids_yaml': LaunchConfiguration('frame_ids_yaml'),
                'xela_server_exec': LaunchConfiguration('xela_server_exec'),
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
