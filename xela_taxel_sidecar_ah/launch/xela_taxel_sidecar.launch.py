from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    cpp_launch = PathJoinSubstitution([
        FindPackageShare("xela_taxel_sidecar_ah"),
        "launch",
        "xela_taxel_sidecar_cpp.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("in_topic", default_value="/x_taxel_ah"),
        DeclareLaunchArgument("out_topic", default_value="/x_taxel_ah/web_state"),
        DeclareLaunchArgument("viz_mode", default_value="grid"),
        DeclareLaunchArgument("model_name", default_value="XR23AHLCPP"),
        DeclareLaunchArgument("hand_side", default_value="auto"),
        DeclareLaunchArgument("web_port", default_value="8765"),
        DeclareLaunchArgument("sidecar_rosbridge_port", default_value="9090"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cpp_launch),
            launch_arguments={
                "in_topic": LaunchConfiguration("in_topic"),
                "out_topic": LaunchConfiguration("out_topic"),
                "viz_mode": LaunchConfiguration("viz_mode"),
                "model_name": LaunchConfiguration("model_name"),
                "hand_side": LaunchConfiguration("hand_side"),
                "web_port": LaunchConfiguration("web_port"),
                "sidecar_rosbridge_port": LaunchConfiguration("sidecar_rosbridge_port"),
            }.items(),
        ),
    ])
