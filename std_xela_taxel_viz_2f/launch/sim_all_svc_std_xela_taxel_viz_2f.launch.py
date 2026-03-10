from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _sidecar_model_overrides(model_name: str):
    normalized = model_name.strip().lower()
    urdf_sign_overrides = {
        'urdf_left_force_x_sign': '-1.0',
        'urdf_left_force_y_sign': '1.0',
        'urdf_right_force_x_sign': '-1.0',
        'urdf_right_force_y_sign': '1.0',
    }
    if normalized == 'usprds':
        return {
            'grid_rows': '4',
            'grid_cols': '8',
            'row_flip_right': 'false',
            'col_flip_right': 'false',
            **urdf_sign_overrides,
        }
    if normalized == 'usprhe35':
        return {
            'grid_rows': '3',
            'grid_cols': '5',
            **urdf_sign_overrides,
        }
    return {}


def generate_launch_description():
    viz_share = FindPackageShare('std_xela_taxel_viz_2f')
    sidecar_share = FindPackageShare('xela_taxel_sidecar_2f')
    server_share = FindPackageShare('xela_server2_2f')
    replayer_share = FindPackageShare('sim_xela_server')

    grid_rviz = PathJoinSubstitution([viz_share, 'config', 'rviz', 'grid_xela_taxel_viz_2f.rviz'])
    urdf_rviz = PathJoinSubstitution([viz_share, 'config', 'rviz', 'urdf_xela_taxel_viz_2f.rviz'])
    server_launch = PathJoinSubstitution([server_share, 'launch', 'xela_server2_2f_with_replayer.launch.py'])
    default_frame_ids = PathJoinSubstitution([server_share, 'config', 'server2_2f_config.yaml'])
    default_replayer_params = PathJoinSubstitution([replayer_share, 'config', 'replayer_presets.yaml'])
    viz_launch = PathJoinSubstitution([viz_share, 'launch', 'std_xela_taxel_viz_2f.launch.py'])
    sidecar_launch = PathJoinSubstitution([sidecar_share, 'launch', 'xela_taxel_sidecar_cpp.launch.py'])
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

    def build_sidecar_include(context):
        ns_value = LaunchConfiguration('namespace').perform(context).strip()
        viz_node_name = f'/{ns_value}/std_xela_taxel_viz_2f' if ns_value else '/std_xela_taxel_viz_2f'
        tf_topic = f'/{ns_value}/tf' if ns_value else '/tf'
        tf_static_topic = f'/{ns_value}/tf_static' if ns_value else '/tf_static'
        model_name = LaunchConfiguration('viz_model_name').perform(context)

        sidecar_args = {
            'in_topic': '/x_taxel_2f',
            'out_topic': LaunchConfiguration('sidecar_out_topic').perform(context),
            'viz_mode': LaunchConfiguration('viz_mode').perform(context),
            'model_name': model_name,
            'style_preset': LaunchConfiguration('style_preset').perform(context),
            'viz_node_name': viz_node_name,
            'bridge_tf_topic': tf_topic,
            'bridge_tf_static_topic': tf_static_topic,
            'enable_web_server': LaunchConfiguration('sidecar_enable_web_server').perform(context),
            'web_host': LaunchConfiguration('sidecar_web_host').perform(context),
            'web_port': LaunchConfiguration('sidecar_web_port').perform(context),
            'enable_sidecar_rosbridge': LaunchConfiguration('sidecar_enable_rosbridge').perform(context),
            'sidecar_rosbridge_host': LaunchConfiguration('sidecar_rosbridge_host').perform(context),
            'sidecar_rosbridge_port': LaunchConfiguration('sidecar_rosbridge_port').perform(context),
        }
        sidecar_args.update(_sidecar_model_overrides(model_name))

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sidecar_launch),
                launch_arguments=sidecar_args.items(),
                condition=IfCondition(LaunchConfiguration('enable_sidecar')),
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='xviz2f'),
        DeclareLaunchArgument('viz_model_name', default_value='uSPr2F'),
        DeclareLaunchArgument('viz_mode', default_value='grid'),
        DeclareLaunchArgument('style_preset', default_value='default'),
        DeclareLaunchArgument('overlay_grid_in_urdf', default_value='false'),
        DeclareLaunchArgument('joint_states_mode', default_value='local'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('marker_stamp_mode', default_value='now'),
        DeclareLaunchArgument('marker_time_offset_sec', default_value='-0.12'),
        DeclareLaunchArgument('model_params_file', default_value=default_model_params_file),
        DeclareLaunchArgument('rviz_config', default_value=''),

        DeclareLaunchArgument('server_model_name', default_value=LaunchConfiguration('viz_model_name')),
        DeclareLaunchArgument('bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('bind_port', default_value='5000'),
        DeclareLaunchArgument('ws_host', default_value='localhost'),
        DeclareLaunchArgument('ws_port', default_value='5000'),
        DeclareLaunchArgument('frame_ids_yaml', default_value=default_frame_ids),
        DeclareLaunchArgument('warmup_sec', default_value='5.0'),
        DeclareLaunchArgument('preset', default_value='normal'),
        DeclareLaunchArgument('publish_period_ms', default_value='200'),
        DeclareLaunchArgument('sine_freq_hz', default_value='0.05'),
        DeclareLaunchArgument('integer_z_range', default_value='10085.0'),
        DeclareLaunchArgument('calib_z_range', default_value='14.0'),
        DeclareLaunchArgument('z_bias_power', default_value='1.0'),
        DeclareLaunchArgument('replayer_params_file', default_value=default_replayer_params),
        DeclareLaunchArgument('enable_sidecar', default_value='false'),
        DeclareLaunchArgument('sidecar_out_topic', default_value='/x_taxel_2f/web_state'),
        DeclareLaunchArgument('sidecar_enable_web_server', default_value='true'),
        DeclareLaunchArgument('sidecar_web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('sidecar_web_port', default_value='8765'),
        DeclareLaunchArgument('sidecar_enable_rosbridge', default_value='true'),
        DeclareLaunchArgument('sidecar_rosbridge_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('sidecar_rosbridge_port', default_value='9090'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(server_launch),
            launch_arguments={
                'model_name': LaunchConfiguration('server_model_name'),
                'bind_host': LaunchConfiguration('bind_host'),
                'bind_port': LaunchConfiguration('bind_port'),
                'ws_host': LaunchConfiguration('ws_host'),
                'ws_port': LaunchConfiguration('ws_port'),
                'frame_ids_yaml': LaunchConfiguration('frame_ids_yaml'),
                'warmup_sec': LaunchConfiguration('warmup_sec'),
                'preset': LaunchConfiguration('preset'),
                'publish_period_ms': LaunchConfiguration('publish_period_ms'),
                'sine_freq_hz': LaunchConfiguration('sine_freq_hz'),
                'integer_z_range': LaunchConfiguration('integer_z_range'),
                'calib_z_range': LaunchConfiguration('calib_z_range'),
                'z_bias_power': LaunchConfiguration('z_bias_power'),
                'replayer_params_file': LaunchConfiguration('replayer_params_file'),
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
        OpaqueFunction(function=build_sidecar_include),

        OpaqueFunction(function=build_rviz_node),
    ])
