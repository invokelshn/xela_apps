import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('std_xela_taxel_viz_dg5f'),
        'config',
        'base',
        'std_xela_taxel_viz_dg5f.yaml'
    ])

    urdf_xacro = PathJoinSubstitution([
        FindPackageShare('std_xela_taxel_viz_dg5f'),
        'description',
        'xela_dg5f_r_1_module.xacro',
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            LaunchConfiguration('urdf_xacro_path'), ' ',
            'use_ros2_control:=false ',
            'hand_side:=', LaunchConfiguration('hand_side'),
        ]),
        value_type=str
    )

    # x_dg5f_hand's own finger joints (rj_dg_*, ~20 revolute joints) need a
    # joint_states source or robot_state_publisher will never complete their
    # part of the TF tree (world->rl_dg_palm works via fixed joints, but
    # nothing downstream of a revolute joint gets published without one).
    # Unlike AH -- whose real ros2_control hardware interface already feeds
    # /joint_states for its finger joints -- this standalone viz hand has no
    # such driver, so a generic joint_state_publisher (publishing default 0
    # positions) fills that gap. It reads a taxels:=0 rendering of the SAME
    # xacro so it never learns about the 124 taxel dot joints at all,
    # avoiding any conflict with std_xela_joint_state_publisher_node below
    # (which is the one actually driving those from live sensor data).
    robot_description_no_taxels = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            LaunchConfiguration('urdf_xacro_path'), ' ',
            'use_ros2_control:=false ',
            'hand_side:=', LaunchConfiguration('hand_side'), ' ',
            'taxels:=0',
        ]),
        value_type=str
    )

    def build_viz_node(context):
        viz_mode_value = LaunchConfiguration('viz_mode').perform(context)
        model_name = LaunchConfiguration('model_name').perform(context)
        overlay_value = LaunchConfiguration('overlay_grid_in_urdf').perform(context)
        overlay_bool = str(overlay_value).lower() in ('1', 'true', 'yes', 'on')
        style_value = LaunchConfiguration('style_preset').perform(context)
        marker_stamp_mode = LaunchConfiguration('marker_stamp_mode').perform(context)
        marker_time_offset_raw = LaunchConfiguration('marker_time_offset_sec').perform(context)
        try:
            marker_time_offset = float(marker_time_offset_raw)
        except (TypeError, ValueError):
            marker_time_offset = 0.0
        frame_id_value = LaunchConfiguration('frame_id').perform(context).strip()
        frame_prefix_value = LaunchConfiguration('frame_prefix').perform(context).strip()
        if frame_prefix_value.startswith('/'):
            frame_prefix_value = frame_prefix_value.lstrip('/')

        pkg_share = get_package_share_directory('std_xela_taxel_viz_dg5f')
        # Model dir scheme mirrors std_xela_taxel_viz_2f's config/models/<model_name>/,
        # minus 2F's mount_type subdirectory (not applicable to DG-5F).
        grid_config_path = os.path.join(pkg_share, 'config', 'models', model_name, 'grid.yaml')

        overrides = {
            'viz_mode': viz_mode_value,
            'overlay_grid_in_urdf': overlay_bool,
            'style_preset': style_value,
            'marker_stamp_mode': marker_stamp_mode,
            'marker_time_offset_sec': marker_time_offset,
            'model_name': model_name,
            'frame_id': frame_id_value,
            'frame_prefix': frame_prefix_value,
            'in_topic': '/x_taxel_dg5f',
            'out_topic': 'markers',
            'grid_config_yaml': grid_config_path,
        }

        # Merge model grid.yaml's ros__parameters (canvas_rows/cols, cell_size, etc.)
        # explicitly, same reasoning as std_xela_taxel_viz_2f: guarantees these
        # override the base yaml regardless of ROS parameter file load order.
        if os.path.isfile(grid_config_path):
            try:
                with open(grid_config_path, 'r') as f:
                    model_yaml = yaml.safe_load(f)
                ros_params = (model_yaml or {}).get('/**', {}).get('ros__parameters', {})
                overrides.update(ros_params)
            except Exception:
                pass
        # grid_config_yaml must win over anything the model yaml itself sets for it.
        overrides['grid_config_yaml'] = grid_config_path

        return [
            Node(
                package='std_xela_taxel_viz_dg5f',
                executable='std_xela_taxel_viz_dg5f_node',
                name='std_xela_taxel_viz_dg5f',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    overrides,
                ],
            )
        ]

    def build_robot_state_publisher(context):
        ns_value = LaunchConfiguration('namespace').perform(context).strip()
        joint_states_mode = LaunchConfiguration('joint_states_mode').perform(context).strip().lower()
        if joint_states_mode == 'global':
            joint_states_topic = '/joint_states'
        else:
            joint_states_topic = f'/{ns_value}/joint_states' if ns_value else '/joint_states'

        frame_prefix_value = LaunchConfiguration('frame_prefix').perform(context).strip()
        if frame_prefix_value.startswith('/'):
            frame_prefix_value = frame_prefix_value.lstrip('/')

        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[
                    {
                        'robot_description': robot_description,
                        'frame_prefix': frame_prefix_value,
                    }
                ],
                remappings=[
                    ('joint_states', joint_states_topic),
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ],
            )
        ]

    use_local_joint_states = PythonExpression(
        ["'true' if '", LaunchConfiguration('joint_states_mode'), "' == 'local' else 'false'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='xvizdg5f',
            description='Namespace for std_xela_taxel_viz_dg5f nodes.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the base parameters file.'
        ),
        DeclareLaunchArgument(
            'hand_side',
            default_value='right',
            description='DG-5F hand side. Only "right" is supported by this package for now.'
        ),
        DeclareLaunchArgument(
            'viz_mode',
            default_value='grid',
            description='Visualization mode: grid or urdf.'
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='XDG5FR',
            description='Model name used for grid.yaml selection under config/models/.',
        ),
        DeclareLaunchArgument(
            'overlay_grid_in_urdf',
            default_value='false',
            description='Overlay the grid when viz_mode=urdf.'
        ),
        DeclareLaunchArgument(
            'style_preset',
            default_value='default',
            description='Grid style preset.'
        ),
        DeclareLaunchArgument(
            'marker_stamp_mode',
            default_value='now',
            description='Marker stamp mode: keep | now | zero.'
        ),
        DeclareLaunchArgument(
            'marker_time_offset_sec',
            default_value='-0.12',
            description='Marker timestamp offset in seconds.'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='world',
            description='Marker frame_id for grid mode.'
        ),
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='TF frame prefix (leave empty for local /tf remap).'
        ),
        DeclareLaunchArgument(
            'urdf_xacro_path',
            default_value=urdf_xacro,
            description='Path to the xacro file for URDF mode.'
        ),
        DeclareLaunchArgument(
            'joint_states_mode',
            default_value='local',
            description='joint_states mode: global or local.'
        ),
        DeclareLaunchArgument(
            'joint_states_config_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('std_xela_taxel_viz_dg5f'),
                'config',
                'joints',
                'joint_state_profiles.yaml',
            ]),
            description='Config YAML for taxel joint list.'
        ),
        DeclareLaunchArgument(
            'joint_states_device_profile',
            default_value=LaunchConfiguration('model_name'),
            description='Device profile for taxel joint list.'
        ),
        OpaqueFunction(function=build_viz_node),
        OpaqueFunction(function=build_robot_state_publisher),
        Node(
            package='std_xela_taxel_viz_dg5f',
            executable='std_xela_joint_state_publisher_node',
            name='std_xela_joint_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {
                    'output_topic': 'joint_states',
                    'publish_rate': 30.0,
                    'config_yaml': LaunchConfiguration('joint_states_config_yaml'),
                    'device_profile': LaunchConfiguration('joint_states_device_profile'),
                }
            ],
            condition=IfCondition(use_local_joint_states),
        ),
        # Fills in the ~20 real dg5f_hand finger revolute joints (rj_dg_*)
        # with default (0) positions -- see this file's comment above
        # robot_description_no_taxels for why a separate taxels:=0 rendering
        # is used here (keeps this node from ever seeing/racing the 124
        # taxel dot joints, which std_xela_joint_state_publisher_node above
        # drives from live sensor data).
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'robot_description': robot_description_no_taxels},
            ],
            condition=IfCondition(use_local_joint_states),
        ),
    ])
