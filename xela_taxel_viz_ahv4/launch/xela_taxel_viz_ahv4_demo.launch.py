from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to xela_taxel_viz_ahv4 parameters file.'
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
            'publish_rate_hz',
            default_value='20.0',
            description='Demo publisher rate (Hz).'
        ),
        Node(
            package='xela_taxel_viz_ahv4',
            executable='xela_taxel_viz_ahv4_demo_node',
            name='xela_taxel_viz_ahv4_demo',
            output='screen',
            parameters=[
                {
                    'out_topic': '/x_taxel_ah',
                    'mapping_yaml': LaunchConfiguration('mapping_yaml'),
                    'hand_side': LaunchConfiguration('hand_side'),
                    'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                },
            ],
        ),
        Node(
            package='xela_taxel_viz_ahv4',
            executable='xela_taxel_viz_ahv4_node',
            name='xela_taxel_viz_ahv4',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'mapping_yaml': LaunchConfiguration('mapping_yaml'),
                    'hand_side': LaunchConfiguration('hand_side'),
                    'pattern_yaml': LaunchConfiguration('pattern_yaml'),
                },
            ],
        ),
    ])
