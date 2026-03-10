from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    viz_share = FindPackageShare('std_xela_taxel_viz_2f')
    server_share = FindPackageShare('xela_server2_2f')

    grid_rviz = PathJoinSubstitution([viz_share, 'config', 'rviz', 'grid_xela_taxel_viz_2f.rviz'])
    urdf_rviz = PathJoinSubstitution([viz_share, 'config', 'rviz', 'urdf_xela_taxel_viz_2f.rviz'])
    server_launch = PathJoinSubstitution([server_share, 'launch', 'xela_server2_2f_with_server.launch.py'])
    default_frame_ids = PathJoinSubstitution([server_share, 'config', 'server2_2f_config.yaml'])
    viz_launch = PathJoinSubstitution([viz_share, 'launch', 'std_xela_taxel_viz_2f.launch.py'])
    default_model_params_file = PathJoinSubstitution([
        viz_share,
        'config',
        'models',
        LaunchConfiguration('viz_model_name'),
        PythonExpression(["'", LaunchConfiguration('viz_mode'), "' + '.yaml'"]),
    ])

    def build_rviz_node(context):
        viz_mode = LaunchConfiguration('viz_mode').perform(context)
        cfg_override = LaunchConfiguration('rviz_config').perform(context).strip()
        cfg = cfg_override or (grid_rviz if viz_mode == 'grid' else urdf_rviz)
        ns_value = LaunchConfiguration('namespace').perform(context).strip()
        tf_topic = f'/{ns_value}/tf' if ns_value else '/tf'
        tf_static_topic = f'/{ns_value}/tf_static' if ns_value else '/tf_static'
        return [
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', cfg],
                remappings=[
                    ('/tf', tf_topic),
                    ('/tf_static', tf_static_topic),
                ],
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='xviz2f'),
        DeclareLaunchArgument('viz_model_name', default_value='uSPr2F'),
        DeclareLaunchArgument('viz_mode', default_value='urdf'),
        DeclareLaunchArgument('style_preset', default_value='default'),
        DeclareLaunchArgument('overlay_grid_in_urdf', default_value='false'),
        DeclareLaunchArgument('joint_states_mode', default_value='local'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('marker_stamp_mode', default_value='now'),
        DeclareLaunchArgument('marker_time_offset_sec', default_value='-0.12'),
        DeclareLaunchArgument('model_params_file', default_value=default_model_params_file),
        DeclareLaunchArgument('rviz_config', default_value=''),

        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument('ws_host', default_value='localhost'),
        DeclareLaunchArgument('ws_port', default_value='5000'),
        DeclareLaunchArgument('frame_ids_yaml', default_value=default_frame_ids),
        DeclareLaunchArgument('xela_server_exec', default_value='/etc/xela/xela_server'),

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
                'namespace': LaunchConfiguration('namespace'),
                'model_name': LaunchConfiguration('viz_model_name'),
                'viz_mode': LaunchConfiguration('viz_mode'),
                'style_preset': LaunchConfiguration('style_preset'),
                'overlay_grid_in_urdf': LaunchConfiguration('overlay_grid_in_urdf'),
                'joint_states_mode': LaunchConfiguration('joint_states_mode'),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
                'marker_stamp_mode': LaunchConfiguration('marker_stamp_mode'),
                'marker_time_offset_sec': LaunchConfiguration('marker_time_offset_sec'),
                'model_params_file': LaunchConfiguration('model_params_file'),
            }.items(),
        ),

        OpaqueFunction(function=build_rviz_node),
    ])
