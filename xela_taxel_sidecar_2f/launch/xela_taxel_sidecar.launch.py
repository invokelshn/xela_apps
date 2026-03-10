from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("xela_taxel_sidecar_2f")
    bridge_script = PathJoinSubstitution([pkg, "scripts", "xela_taxel_web_bridge.py"])
    web_root = PathJoinSubstitution([pkg, "web", "taxel_sidecar"])

    bridge = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            bridge_script,
            "--ros-args",
            "-p", ["in_topic:=", LaunchConfiguration("in_topic")],
            "-p", ["out_topic:=", LaunchConfiguration("out_topic")],
            "-p", ["viz_mode:=", LaunchConfiguration("viz_mode")],
            "-p", ["model_name:=", LaunchConfiguration("model_name")],
            "-p", ["fixed_frame:=", LaunchConfiguration("fixed_frame")],
            "-p", ["left_module_index:=", LaunchConfiguration("left_module_index")],
            "-p", ["right_module_index:=", LaunchConfiguration("right_module_index")],
            "-p", ["grid_rows:=", LaunchConfiguration("grid_rows")],
            "-p", ["grid_cols:=", LaunchConfiguration("grid_cols")],
            "-p", ["cell_size:=", LaunchConfiguration("cell_size")],
            "-p", ["module_gap:=", LaunchConfiguration("module_gap")],
            "-p", ["row_flip_right:=", LaunchConfiguration("row_flip_right")],
            "-p", ["col_flip_right:=", LaunchConfiguration("col_flip_right")],
            "-p", ["max_publish_rate_hz:=", LaunchConfiguration("max_publish_rate_hz")],
            "-p", ["emit_urdf_points:=", LaunchConfiguration("emit_urdf_points")],
            "-p", ["freeze_urdf_positions:=", LaunchConfiguration("freeze_urdf_positions")],
        ],
        output="screen",
    )

    web_server = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            PathJoinSubstitution([pkg, "scripts", "sidecar_http_server.py"]),
            "--host",
            LaunchConfiguration("web_host"),
            "--port",
            LaunchConfiguration("web_port"),
            "--web-root",
            web_root,
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_web_server")),
    )

    sidecar_rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket_sidecar",
        parameters=[
            {
                "port": LaunchConfiguration("sidecar_rosbridge_port"),
                "address": LaunchConfiguration("sidecar_rosbridge_host"),
                "max_message_size": 1000000000000,
                "call_services_in_new_thread": True,
                "send_action_goals_in_new_thread": True,
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_sidecar_rosbridge")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("in_topic", default_value="/x_taxel_2f"),
        DeclareLaunchArgument("out_topic", default_value="/x_taxel_2f/web_state"),
        DeclareLaunchArgument("viz_mode", default_value="grid"),
        DeclareLaunchArgument("model_name", default_value="uSPr2F"),
        DeclareLaunchArgument("fixed_frame", default_value="world"),
        DeclareLaunchArgument("left_module_index", default_value="0"),
        DeclareLaunchArgument("right_module_index", default_value="1"),
        DeclareLaunchArgument("grid_rows", default_value="4"),
        DeclareLaunchArgument("grid_cols", default_value="6"),
        DeclareLaunchArgument("cell_size", default_value="0.015"),
        DeclareLaunchArgument("module_gap", default_value="0.04"),
        DeclareLaunchArgument("row_flip_right", default_value="true"),
        DeclareLaunchArgument("col_flip_right", default_value="true"),
        DeclareLaunchArgument("max_publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("emit_urdf_points", default_value="false"),
        DeclareLaunchArgument("freeze_urdf_positions", default_value="true"),
        DeclareLaunchArgument("enable_web_server", default_value="true"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8765"),
        DeclareLaunchArgument("enable_sidecar_rosbridge", default_value="true"),
        DeclareLaunchArgument("sidecar_rosbridge_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("sidecar_rosbridge_port", default_value="9090"),
        sidecar_rosbridge,
        bridge,
        web_server,
    ])
