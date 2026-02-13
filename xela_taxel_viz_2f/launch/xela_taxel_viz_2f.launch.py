from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression, TextSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('xela_taxel_viz_2f'),
        'config',
        'base.yaml'
    ])

    urdf_xacro = PathJoinSubstitution([
        FindPackageShare('xela_taxel_viz_2f'),
        'description',
        PythonExpression([
            "'xela_' + '", LaunchConfiguration('model_name'), "' + '_2_modules.xacro'"
        ]),
    ])

    urdf_mode = PythonExpression(["'", LaunchConfiguration('viz_mode'), "' == 'urdf'"])
    use_ros2_control = PythonExpression(
        ["'true' if '", LaunchConfiguration('viz_mode'), "' == 'urdf' else 'false'"]
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            LaunchConfiguration('urdf_xacro_path'), ' ',
            'use_ros2_control:=', use_ros2_control
        ]),
        value_type=str
    )

    def build_ros2_control_node(context):
        model_name = LaunchConfiguration('model_name').perform(context)
        controllers_yaml = f"{get_package_share_directory('xela_taxel_viz_2f')}/config/models/{model_name}/ros2_controllers.yaml"
        return [
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                output='screen',
                parameters=[
                    {'robot_description': robot_description},
                    controllers_yaml,
                ],
                condition=IfCondition(urdf_mode),
            )
        ]

    def build_viz_node(context):
        viz_mode_value = LaunchConfiguration('viz_mode').perform(context)
        model_name = LaunchConfiguration('model_name').perform(context)
        overlay_value = LaunchConfiguration('overlay_grid_in_urdf').perform(context)
        overlay_bool = str(overlay_value).lower() in ('1', 'true', 'yes', 'on')
        style_value = LaunchConfiguration('style_preset').perform(context)
        params_file_value = LaunchConfiguration('params_file').perform(context)
        model_params_file_value = LaunchConfiguration('model_params_file').perform(context)

        overrides = {
            'viz_mode': viz_mode_value,
            'overlay_grid_in_urdf': overlay_bool,
            'style_preset': style_value,
            'model_name': model_name,
        }

        return [
            Node(
                package='xela_taxel_viz_2f',
                executable='xela_taxel_viz_2f_node',
                name='xela_taxel_viz_2f',
                output='screen',
                parameters=[
                    params_file_value,
                    model_params_file_value,
                    overrides,
                ],
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the base parameters file.'
        ),
        DeclareLaunchArgument(
            'model_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('xela_taxel_viz_2f'),
                'config',
                'models',
                LaunchConfiguration('model_name'),
                PythonExpression(["'", LaunchConfiguration('viz_mode'), "' + '.yaml'"]),
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
            description='Model name used for URDF and device profile selection.',
        ),
        DeclareLaunchArgument(
            'overlay_grid_in_urdf',
            default_value='false',
            description='Overlay the grid when viz_mode=urdf.'
        ),
        DeclareLaunchArgument(
            'style_preset',
            default_value='default',
            description='Grid style preset: default, cool_steel, deep_navy, warm_graphite.',
        ),
        DeclareLaunchArgument(
            'urdf_xacro_path',
            default_value=urdf_xacro,
            description='Path to the xacro file for URDF mode.'
        ),
        OpaqueFunction(function=build_viz_node),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(urdf_mode),
        ),
        OpaqueFunction(function=build_ros2_control_node),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['xela_taxel_joint_state_publisher', '--controller-manager', '/controller_manager'],
            output='screen',
            condition=IfCondition(urdf_mode),
        ),
    ])
