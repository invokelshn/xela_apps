from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
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
        FindPackageShare('xela_taxel_viz_ahv4'),
        'config',
        'xela_taxel_viz_ahv4.yaml'
    ])

    mapping_yaml = PathJoinSubstitution([
        FindPackageShare('xela_taxel_viz_ahv4'),
        'config',
        'taxel_joint_map_new.yaml'
    ])

    pattern_yaml = PathJoinSubstitution([
        FindPackageShare('xela_taxel_viz_ahv4'),
        'config',
        'pattern_lahv4.yaml'
    ])

    urdf_xacro = PathJoinSubstitution([
        FindPackageShare('xela_taxel_viz_ahv4'),
        'description',
        'xela_uSCuAH_0_modules.xacro'
    ])

    ros2_controller_yaml = PathJoinSubstitution([
        FindPackageShare('xela_taxel_viz_ahv4'),
        'config',
        'ros2_controller_xela_taxel_viz_ahv4.yaml'
    ])

    urdf_mode = PythonExpression(["'", LaunchConfiguration('viz_mode'), "' == 'urdf'"])
    use_hand_controllers = PythonExpression(
        ["'", LaunchConfiguration('use_hand_controllers'), "' == 'true'"]
    )
    use_taxel_ros2_control = PythonExpression(
        ["'", LaunchConfiguration('viz_mode'), "' == 'urdf' and '",
         LaunchConfiguration('use_hand_controllers'), "' == 'false'"]
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            LaunchConfiguration('urdf_xacro_path'), ' ',
            'sequence:=', LaunchConfiguration('sequence'), ' ',
            'tips:=', LaunchConfiguration('tips'), ' ',
            'parent:=', LaunchConfiguration('parent'), ' ',
            'use_ros2_control:=', use_taxel_ros2_control, ' ',
            'use_hand_controllers:=', use_hand_controllers,
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
        overrides = {
            'frame_id': frame_id_value,
            'viz_mode': viz_mode_value,
            'overlay_grid_in_urdf': overlay_bool,
            'marker_stamp_mode': marker_stamp_mode,
            'marker_time_offset_sec': marker_time_offset,
        }

        tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.safe_dump({'/**': {'ros__parameters': overrides}}, tmp)
        tmp.flush()
        tmp.close()

        return [
            Node(
                package='xela_taxel_viz_ahv4',
                executable='xela_taxel_viz_ahv4_node',
                name='xela_taxel_viz_ahv4',
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

    def build_jsp_override(context):
        device_profile = LaunchConfiguration('jsp_device_profile').perform(context).strip()
        hand_side = LaunchConfiguration('jsp_hand_side').perform(context).strip()
        if not device_profile and not hand_side:
            return []

        overrides = {
            'xela_taxel_joint_state_publisher': {
                'ros__parameters': {
                    'device_profile': device_profile,
                    'hand_side': hand_side,
                }
            }
        }

        tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.safe_dump(overrides, tmp)
        tmp.flush()
        tmp.close()

        return [
            SetLaunchConfiguration('jsp_override_file', tmp.name),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to xela_taxel_viz_ahv4 parameters file.'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='x_taxel_ah_viz',
            description='Marker frame_id for grid mode (e.g., world).'
        ),
        DeclareLaunchArgument(
            'viz_mode',
            default_value='urdf',
            description='Visualization mode: grid or urdf.'
        ),
        DeclareLaunchArgument(
            'use_hand_controllers',
            default_value='true',
            description='Start Allegro hand controllers + joint_state_broadcaster.'
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
            description='Marker timestamp offset in seconds (negative to avoid TF future).'
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
            'hand_ros2_control_controllers_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('xela_taxel_viz_ahv4'),
                'config',
                'ros2_controller_hand_xela_taxel_viz_ahv4.yaml',
            ]),
            description='Controllers YAML containing hand_controller + joint_state_broadcaster.'
        ),
        DeclareLaunchArgument(
            'jsp_device_profile',
            default_value='uSCuAH_left',
            description='Device profile for xela_taxel_joint_state_publisher.'
        ),
        DeclareLaunchArgument(
            'jsp_hand_side',
            default_value='left',
            description='Hand side for xela_taxel_joint_state_publisher: left or right.'
        ),
        DeclareLaunchArgument(
            'jsp_override_file',
            default_value='',
            description='Internal override file for xela_taxel_joint_state_publisher.'
        ),
        OpaqueFunction(function=build_viz_node),
        OpaqueFunction(function=build_jsp_override),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(urdf_mode),
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                ros2_controller_yaml,
            ],
            condition=IfCondition(use_taxel_ros2_control),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['xela_taxel_joint_state_publisher', '--controller-manager', '/controller_manager'],
            output='screen',
            condition=IfCondition(use_taxel_ros2_control),
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                LaunchConfiguration('hand_ros2_control_controllers_yaml'),
                ros2_controller_yaml,
                LaunchConfiguration('jsp_override_file'),
            ],
            condition=IfCondition(use_hand_controllers),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
            condition=IfCondition(use_hand_controllers),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['hand_controller', '--controller-manager', '/controller_manager'],
            output='screen',
            condition=IfCondition(use_hand_controllers),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['xela_taxel_joint_state_publisher', '--controller-manager', '/controller_manager'],
            output='screen',
            condition=IfCondition(use_hand_controllers),
        ),
    ])
