from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    viz_share = FindPackageShare('std_xela_taxel_viz_ahv4')
    sidecar_share = FindPackageShare('xela_taxel_sidecar_ah')
    server_share = FindPackageShare('xela_server2_ah')
    replayer_share = FindPackageShare('sim_xela_server')

    rviz_grid_cfg = PathJoinSubstitution([viz_share, 'config', 'rviz', 'grid_xela_taxel_viz_ah.rviz'])
    rviz_urdf_cfg = PathJoinSubstitution([viz_share, 'config', 'rviz', 'urdf_xela_taxel_viz_ah.rviz'])
    server_launch = PathJoinSubstitution([server_share, 'launch', 'xela_server2_ah_with_mock_server.launch.py'])
    default_frame_ids = PathJoinSubstitution([server_share, 'config', 'server2_ah_config.yaml'])
    default_replayer_params = PathJoinSubstitution([replayer_share, 'config', 'replayer_presets.yaml'])
    viz_launch = PathJoinSubstitution([viz_share, 'launch', 'std_xela_taxel_viz_ahv4.launch.py'])
    sidecar_launch = PathJoinSubstitution([sidecar_share, 'launch', 'xela_taxel_sidecar_cpp.launch.py'])
    urdf_left = PathJoinSubstitution([viz_share, 'description', 'xela_uSCuAH_0_modules.xacro'])
    urdf_right = PathJoinSubstitution([viz_share, 'description', 'xela_uSCuAH_1_modules.xacro'])
    pattern_left = PathJoinSubstitution([viz_share, 'config', 'patterns', 'pattern_lahv4.yaml'])
    pattern_right = PathJoinSubstitution([viz_share, 'config', 'patterns', 'pattern_rahv4.yaml'])
    mapping_default = PathJoinSubstitution([viz_share, 'config', 'maps', 'taxel_joint_map_new.yaml'])

    def build_rviz_node(context):
        cfg_override = LaunchConfiguration('rviz_config').perform(context).strip()
        viz_mode = LaunchConfiguration('viz_mode').perform(context).strip()
        default_cfg = rviz_grid_cfg if viz_mode == 'grid' else rviz_urdf_cfg
        cfg = cfg_override or default_cfg
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
        viz_node_name = f'/{ns_value}/std_xela_taxel_viz_ahv4' if ns_value else '/std_xela_taxel_viz_ahv4'
        tf_topic = f'/{ns_value}/tf' if ns_value else '/tf'
        tf_static_topic = f'/{ns_value}/tf_static' if ns_value else '/tf_static'

        sidecar_args = {
            'in_topic': '/x_taxel_ah',
            'out_topic': LaunchConfiguration('sidecar_out_topic').perform(context),
            'viz_mode': LaunchConfiguration('viz_mode').perform(context),
            'model_name': LaunchConfiguration('model_name').perform(context),
            'hand_side': LaunchConfiguration('hand_side').perform(context),
            'mapping_yaml': LaunchConfiguration('mapping_yaml').perform(context),
            'pattern_yaml': LaunchConfiguration('pattern_yaml').perform(context),
            'style_preset': 'cool_steel',
            'fixed_frame': LaunchConfiguration('frame_id').perform(context),
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
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sidecar_launch),
                launch_arguments=sidecar_args.items(),
            ),
        ]

    def select_hand_description(context):
        model_name = LaunchConfiguration('model_name').perform(context).strip()
        urdf_override = LaunchConfiguration('urdf_xacro_path').perform(context).strip()
        seq_override = LaunchConfiguration('sequence').perform(context).strip()
        pattern_override = LaunchConfiguration('pattern_yaml').perform(context).strip()
        mapping_override = LaunchConfiguration('mapping_yaml').perform(context).strip()
        hand_side_override = LaunchConfiguration('hand_side').perform(context).strip()
        is_right = model_name in ('XR23AHRCPP', 'XR23AHRCPP_right')

        urdf_path = urdf_override or (urdf_right if is_right else urdf_left)
        sequence = seq_override or ('1' if is_right else '0')
        pattern_path = pattern_override or (pattern_right if is_right else pattern_left)
        mapping_path = mapping_override or mapping_default
        hand_side = hand_side_override or ('right' if is_right else 'left')
        jsp_profile = 'uSCuAH_right' if hand_side == 'right' else 'uSCuAH_left'

        return [
            SetLaunchConfiguration('urdf_xacro_path', urdf_path),
            SetLaunchConfiguration('sequence', sequence),
            SetLaunchConfiguration('pattern_yaml', pattern_path),
            SetLaunchConfiguration('mapping_yaml', mapping_path),
            SetLaunchConfiguration('hand_side', hand_side),
            SetLaunchConfiguration('joint_states_device_profile', jsp_profile),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='xvizah'),
        DeclareLaunchArgument('model_name', default_value='XR23AHLCPP'),
        DeclareLaunchArgument('viz_mode', default_value='urdf'),
        DeclareLaunchArgument('overlay_grid_in_urdf', default_value='false'),
        DeclareLaunchArgument('joint_states_mode', default_value='local'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('urdf_xacro_path', default_value=''),
        DeclareLaunchArgument('sequence', default_value=''),
        DeclareLaunchArgument('pattern_yaml', default_value=''),
        DeclareLaunchArgument('mapping_yaml', default_value=''),
        DeclareLaunchArgument('hand_side', default_value=''),
        DeclareLaunchArgument('marker_stamp_mode', default_value='keep'),
        DeclareLaunchArgument('marker_time_offset_sec', default_value='-0.1'),
        DeclareLaunchArgument('frame_id', default_value='world'),
        DeclareLaunchArgument('joint_states_device_profile', default_value=''),
        DeclareLaunchArgument('enable_sidecar', default_value='false'),
        DeclareLaunchArgument('sidecar_out_topic', default_value='/x_taxel_ah/web_state'),
        DeclareLaunchArgument('sidecar_enable_web_server', default_value='true'),
        DeclareLaunchArgument('sidecar_web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('sidecar_web_port', default_value='8765'),
        DeclareLaunchArgument('sidecar_enable_rosbridge', default_value='true'),
        DeclareLaunchArgument('sidecar_rosbridge_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('sidecar_rosbridge_port', default_value='9090'),

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

        OpaqueFunction(function=select_hand_description),

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
                'integer_z_range': LaunchConfiguration('integer_z_range'),
                'calib_z_range': LaunchConfiguration('calib_z_range'),
                'z_bias_power': LaunchConfiguration('z_bias_power'),
                'replayer_params_file': LaunchConfiguration('replayer_params_file'),
                'hand_side': LaunchConfiguration('hand_side'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(viz_launch),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'viz_mode': LaunchConfiguration('viz_mode'),
                'overlay_grid_in_urdf': LaunchConfiguration('overlay_grid_in_urdf'),
                'joint_states_mode': LaunchConfiguration('joint_states_mode'),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
                'urdf_xacro_path': LaunchConfiguration('urdf_xacro_path'),
                'sequence': LaunchConfiguration('sequence'),
                'pattern_yaml': LaunchConfiguration('pattern_yaml'),
                'mapping_yaml': LaunchConfiguration('mapping_yaml'),
                'hand_side': LaunchConfiguration('hand_side'),
                'marker_stamp_mode': LaunchConfiguration('marker_stamp_mode'),
                'marker_time_offset_sec': LaunchConfiguration('marker_time_offset_sec'),
                'frame_id': LaunchConfiguration('frame_id'),
                'joint_states_device_profile': LaunchConfiguration('joint_states_device_profile'),
            }.items(),
        ),
        OpaqueFunction(
            function=build_sidecar_include,
            condition=IfCondition(LaunchConfiguration('enable_sidecar')),
        ),
    ])
