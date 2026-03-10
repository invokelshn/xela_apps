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
        FindPackageShare('std_xela_taxel_viz_2f'),
        'config',
        'base',
        'std_xela_taxel_viz_2f.yaml'
    ])

    urdf_xacro = PathJoinSubstitution([
        FindPackageShare('std_xela_taxel_viz_2f'),
        'description',
        PythonExpression([
            "'xela_' + '", LaunchConfiguration('model_name'), "' + '_2_modules.xacro'"
        ]),
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            LaunchConfiguration('urdf_xacro_path'), ' ',
            'use_ros2_control:=false'
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

        overrides = {
            'viz_mode': viz_mode_value,
            'overlay_grid_in_urdf': overlay_bool,
            'style_preset': style_value,
            'marker_stamp_mode': marker_stamp_mode,
            'marker_time_offset_sec': marker_time_offset,
            'model_name': model_name,
            'frame_id': frame_id_value,
            'frame_prefix': frame_prefix_value,
            'in_topic': '/x_taxel_2f',
            'out_topic': 'markers',
        }

        return [
            Node(
                package='std_xela_taxel_viz_2f',
                executable='std_xela_taxel_viz_2f_node',
                name='std_xela_taxel_viz_2f',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    LaunchConfiguration('model_params_file'),
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
            default_value='xviz2f',
            description='Namespace for std_xela_taxel_viz_2f nodes.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the base parameters file.'
        ),
        DeclareLaunchArgument(
            'model_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('std_xela_taxel_viz_2f'),
                'config',
                'models',
                LaunchConfiguration('model_name'),
                PythonExpression(["'", LaunchConfiguration('viz_mode'), "' + '.yaml'"])
            ]),
            description='Path to the model/mode parameters file.'
        ),
        DeclareLaunchArgument(
            'viz_mode',
            default_value='grid',
            description='Visualization mode: grid or urdf.'
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='uSPr2F',
            description='Model name used for URDF selection.',
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
                FindPackageShare('std_xela_taxel_viz_2f'),
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
            package='std_xela_taxel_viz_2f',
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
    ])
