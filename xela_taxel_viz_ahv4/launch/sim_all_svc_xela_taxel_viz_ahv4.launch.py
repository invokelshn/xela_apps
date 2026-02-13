from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    viz_share = FindPackageShare('xela_taxel_viz_ahv4')
    server_share = FindPackageShare('xela_server2_ah')
    replayer_share = FindPackageShare('sim_xela_server')

    rviz_grid_cfg = PathJoinSubstitution([viz_share, 'config', 'grid_xela_taxel_viz_ah.rviz'])
    rviz_urdf_cfg = PathJoinSubstitution([viz_share, 'config', 'urdf_xela_taxel_viz_ah.rviz'])
    server_launch = PathJoinSubstitution([server_share, 'launch', 'xela_server2_ah_with_replayer.launch.py'])
    default_frame_ids = PathJoinSubstitution([server_share, 'config', 'server2_ah_config.yaml'])
    default_replayer_params = PathJoinSubstitution([replayer_share, 'config', 'replayer_presets.yaml'])
    viz_launch = PathJoinSubstitution([viz_share, 'launch', 'xela_taxel_viz_ahv4.launch.py'])
    urdf_left = PathJoinSubstitution([viz_share, 'description', 'xela_uSCuAH_0_modules.xacro'])
    urdf_right = PathJoinSubstitution([viz_share, 'description', 'xela_uSCuAH_1_modules.xacro'])
    pattern_left = PathJoinSubstitution([viz_share, 'config', 'pattern_lahv4.yaml'])
    pattern_right = PathJoinSubstitution([viz_share, 'config', 'pattern_rahv4.yaml'])
    mapping_default = PathJoinSubstitution([viz_share, 'config', 'taxel_joint_map_new.yaml'])

    def build_rviz_node(context):
        cfg_override = LaunchConfiguration('rviz_config').perform(context).strip()
        viz_mode = LaunchConfiguration('viz_mode').perform(context).strip()
        default_cfg = rviz_grid_cfg if viz_mode == 'grid' else rviz_urdf_cfg
        cfg = cfg_override or default_cfg
        return [
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', cfg],
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
            SetLaunchConfiguration('jsp_hand_side', hand_side),
            SetLaunchConfiguration('jsp_device_profile', jsp_profile),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('model_name', default_value='XR23AHLCPP'),
        DeclareLaunchArgument('viz_mode', default_value='urdf'),
        DeclareLaunchArgument('overlay_grid_in_urdf', default_value='false'),
        DeclareLaunchArgument('use_hand_controllers', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('urdf_xacro_path', default_value=''),
        DeclareLaunchArgument('sequence', default_value=''),
        DeclareLaunchArgument('pattern_yaml', default_value=''),
        DeclareLaunchArgument('mapping_yaml', default_value=''),
        DeclareLaunchArgument('hand_side', default_value=''),
        DeclareLaunchArgument('jsp_hand_side', default_value=''),
        DeclareLaunchArgument('jsp_device_profile', default_value=''),
        DeclareLaunchArgument('marker_stamp_mode', default_value='keep'),
        DeclareLaunchArgument('marker_time_offset_sec', default_value='-0.1'),
        DeclareLaunchArgument('frame_id', default_value='x_taxel_ah_viz'),

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
                'replayer_params_file': LaunchConfiguration('replayer_params_file'),
                'hand_side': LaunchConfiguration('hand_side'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(viz_launch),
            launch_arguments={
                'viz_mode': LaunchConfiguration('viz_mode'),
                'overlay_grid_in_urdf': LaunchConfiguration('overlay_grid_in_urdf'),
                'use_hand_controllers': LaunchConfiguration('use_hand_controllers'),
                'urdf_xacro_path': LaunchConfiguration('urdf_xacro_path'),
                'sequence': LaunchConfiguration('sequence'),
                'pattern_yaml': LaunchConfiguration('pattern_yaml'),
                'mapping_yaml': LaunchConfiguration('mapping_yaml'),
                'hand_side': LaunchConfiguration('hand_side'),
                'jsp_hand_side': LaunchConfiguration('jsp_hand_side'),
                'jsp_device_profile': LaunchConfiguration('jsp_device_profile'),
                'marker_stamp_mode': LaunchConfiguration('marker_stamp_mode'),
                'marker_time_offset_sec': LaunchConfiguration('marker_time_offset_sec'),
                'frame_id': LaunchConfiguration('frame_id'),
            }.items(),
        ),

        OpaqueFunction(function=build_rviz_node),
    ])
