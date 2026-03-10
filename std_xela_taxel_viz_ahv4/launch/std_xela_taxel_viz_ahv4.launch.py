from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import tempfile
import yaml


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('std_xela_taxel_viz_ahv4'),
        'config',
        'base',
        'std_xela_taxel_viz_ahv4.yaml'
    ])

    mapping_yaml = PathJoinSubstitution([
        FindPackageShare('std_xela_taxel_viz_ahv4'),
        'config',
        'maps',
        'taxel_joint_map_new.yaml'
    ])

    pattern_yaml = PathJoinSubstitution([
        FindPackageShare('std_xela_taxel_viz_ahv4'),
        'config',
        'patterns',
        'pattern_lahv4.yaml'
    ])

    urdf_xacro = PathJoinSubstitution([
        FindPackageShare('std_xela_taxel_viz_ahv4'),
        'description',
        'xela_uSCuAH_0_modules.xacro'
    ])

    urdf_mode = PythonExpression(["'", LaunchConfiguration('viz_mode'), "' == 'urdf'"])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            LaunchConfiguration('urdf_xacro_path'), ' ',
            'sequence:=', LaunchConfiguration('sequence'), ' ',
            'tips:=', LaunchConfiguration('tips'), ' ',
            'parent:=', LaunchConfiguration('parent'), ' ',
            'use_ros2_control:=false ',
            'use_hand_controllers:=false',
        ]),
        value_type=str
    )

    def build_viz_node(context):
        viz_mode_value = LaunchConfiguration('viz_mode').perform(context)
        overlay_value = LaunchConfiguration('overlay_grid_in_urdf').perform(context)
        overlay_bool = str(overlay_value).lower() in ('1', 'true', 'yes', 'on')
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
        overrides = {
            'frame_id': frame_id_value,
            'frame_prefix': frame_prefix_value,
            'viz_mode': viz_mode_value,
            'overlay_grid_in_urdf': overlay_bool,
            'marker_stamp_mode': marker_stamp_mode,
            'marker_time_offset_sec': marker_time_offset,
            'in_topic': '/x_taxel_ah',
            'out_topic': 'markers',
        }

        tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.safe_dump({'/**': {'ros__parameters': overrides}}, tmp)
        tmp.flush()
        tmp.close()

        return [
            Node(
                package='std_xela_taxel_viz_ahv4',
                executable='std_xela_taxel_viz_ahv4_node',
                name='std_xela_taxel_viz_ahv4',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    tmp.name,
                    {
                        'mapping_yaml': LaunchConfiguration('mapping_yaml'),
                        'hand_side': LaunchConfiguration('hand_side'),
                        'pattern_yaml': LaunchConfiguration('pattern_yaml'),
                    },
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
                condition=IfCondition(urdf_mode),
            )
        ]

    def build_joint_state_publisher(context):
        hand_side = LaunchConfiguration('hand_side').perform(context).strip().lower()
        override_profile = LaunchConfiguration('joint_states_device_profile').perform(context).strip()
        if override_profile:
            device_profile = override_profile
        elif hand_side in ('right', 'r'):
            device_profile = 'uSCuAH_right'
        else:
            device_profile = 'uSCuAH_left'

        extra_joints = [
            'ah_joint00', 'ah_joint01', 'ah_joint02', 'ah_joint03',
            'ah_joint10', 'ah_joint11', 'ah_joint12', 'ah_joint13',
            'ah_joint20', 'ah_joint21', 'ah_joint22', 'ah_joint23',
            'ah_joint30', 'ah_joint31', 'ah_joint32', 'ah_joint33',
        ]

        return [
            Node(
                package='std_xela_taxel_viz_ahv4',
                executable='std_xela_joint_state_publisher_node',
                name='std_xela_joint_state_publisher',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[
                    {
                        'output_topic': 'joint_states',
                        'publish_rate': 30.0,
                        'config_yaml': LaunchConfiguration('joint_states_config_yaml'),
                        'device_profile': device_profile,
                        'extra_joints': extra_joints,
                    }
                ],
                condition=IfCondition(use_local_joint_states),
            )
        ]

    use_local_joint_states = PythonExpression(
        ["'", LaunchConfiguration('viz_mode'), "' == 'urdf' and '",
         LaunchConfiguration('joint_states_mode'), "' == 'local'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='xvizah',
            description='Namespace for std_xela_taxel_viz_ahv4 nodes.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to std_xela_taxel_viz_ahv4 parameters file.'
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
            'viz_mode',
            default_value='grid',
            description='Visualization mode: grid or urdf.'
        ),
        DeclareLaunchArgument(
            'joint_states_mode',
            default_value='local',
            description='joint_states mode: global or local.'
        ),
        DeclareLaunchArgument(
            'overlay_grid_in_urdf',
            default_value='false',
            description='Overlay the grid when viz_mode=urdf.'
        ),
        DeclareLaunchArgument(
            'mapping_yaml',
            default_value=mapping_yaml,
            description='Path to taxel_joint_map_new.yaml.'
        ),
        DeclareLaunchArgument(
            'hand_side',
            default_value='left',
            description='Hand side for mapping prefix: left or right.'
        ),
        DeclareLaunchArgument(
            'pattern_yaml',
            default_value=pattern_yaml,
            description='Path to the 31x26 pattern YAML.'
        ),
        DeclareLaunchArgument(
            'marker_stamp_mode',
            default_value='keep',
            description='Marker stamp mode: keep | now | zero.'
        ),
        DeclareLaunchArgument(
            'marker_time_offset_sec',
            default_value='-0.1',
            description='Marker timestamp offset in seconds.'
        ),
        DeclareLaunchArgument(
            'urdf_xacro_path',
            default_value=urdf_xacro,
            description='Path to the xacro file for URDF mode.'
        ),
        DeclareLaunchArgument(
            'sequence',
            default_value='0',
            description='Sequence number used for taxel prefix (e.g., 0).'
        ),
        DeclareLaunchArgument(
            'tips',
            default_value='curved',
            description='Fingertip type for Allegro hand URDF (curved or flat).'
        ),
        DeclareLaunchArgument(
            'parent',
            default_value='world',
            description='Parent frame for the Allegro hand URDF.'
        ),
        DeclareLaunchArgument(
            'joint_states_config_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('std_xela_taxel_viz_ahv4'),
                'config',
                'joints',
                'joint_state_profiles.yaml',
            ]),
            description='Config YAML for taxel joint list.'
        ),
        DeclareLaunchArgument(
            'joint_states_device_profile',
            default_value='',
            description='Optional override for taxel device profile.'
        ),
        OpaqueFunction(function=build_viz_node),
        OpaqueFunction(function=build_robot_state_publisher),
        OpaqueFunction(function=build_joint_state_publisher),
    ])
