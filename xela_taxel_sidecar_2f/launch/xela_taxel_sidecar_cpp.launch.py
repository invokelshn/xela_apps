from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare("xela_taxel_sidecar_2f")
    web_root = PathJoinSubstitution([pkg, "web", "taxel_sidecar"])

    bridge_node = Node(
        package="xela_taxel_sidecar_2f",
        executable="xela_taxel_web_bridge_node",
        name="xela_taxel_web_bridge_cpp",
        output="screen",
        remappings=[
            ("/tf", LaunchConfiguration("bridge_tf_topic")),
            ("/tf_static", LaunchConfiguration("bridge_tf_static_topic")),
        ],
        parameters=[
            {
                "in_topic": LaunchConfiguration("in_topic"),
                "out_topic": LaunchConfiguration("out_topic"),
                "viz_mode": LaunchConfiguration("viz_mode"),
                "model_name": LaunchConfiguration("model_name"),
                "fixed_frame": LaunchConfiguration("fixed_frame"),
                "left_module_index": LaunchConfiguration("left_module_index"),
                "right_module_index": LaunchConfiguration("right_module_index"),
                "grid_rows": LaunchConfiguration("grid_rows"),
                "grid_cols": LaunchConfiguration("grid_cols"),
                "cell_size": LaunchConfiguration("cell_size"),
                "module_gap": LaunchConfiguration("module_gap"),
                "row_flip_right": LaunchConfiguration("row_flip_right"),
                "col_flip_right": LaunchConfiguration("col_flip_right"),
                "grid_left_force_x_sign": LaunchConfiguration("grid_left_force_x_sign"),
                "grid_left_force_y_sign": LaunchConfiguration("grid_left_force_y_sign"),
                "grid_right_force_x_sign": LaunchConfiguration("grid_right_force_x_sign"),
                "grid_right_force_y_sign": LaunchConfiguration("grid_right_force_y_sign"),
                "urdf_left_force_x_sign": LaunchConfiguration("urdf_left_force_x_sign"),
                "urdf_left_force_y_sign": LaunchConfiguration("urdf_left_force_y_sign"),
                "urdf_right_force_x_sign": LaunchConfiguration("urdf_right_force_x_sign"),
                "urdf_right_force_y_sign": LaunchConfiguration("urdf_right_force_y_sign"),
                "max_publish_rate_hz": LaunchConfiguration("max_publish_rate_hz"),
                "emit_urdf_points": LaunchConfiguration("emit_urdf_points"),
                "freeze_urdf_positions": LaunchConfiguration("freeze_urdf_positions"),
            }
        ],
    )

    mode_manager_node = Node(
        package="xela_taxel_sidecar_2f",
        executable="xela_viz_mode_manager_node",
        name="xela_viz_mode_manager_cpp",
        output="screen",
        parameters=[
            {
                "initial_viz_mode": LaunchConfiguration("viz_mode"),
                "model_name": LaunchConfiguration("model_name"),
                "style_preset": LaunchConfiguration("style_preset"),
                "frame_id": LaunchConfiguration("frame_id"),
                "grid_parent_frame": LaunchConfiguration("grid_parent_frame"),
                "legacy_out_topic": LaunchConfiguration("legacy_out_topic"),
                "max_publish_rate_hz": LaunchConfiguration("max_publish_rate_hz"),
                "publisher_transient_local": LaunchConfiguration("publisher_transient_local"),
                "marker_stamp_mode": LaunchConfiguration("marker_stamp_mode"),
                "circle_marker_type": LaunchConfiguration("circle_marker_type"),
                "managed_launch_package": LaunchConfiguration("managed_launch_package"),
                "managed_launch_file": LaunchConfiguration("managed_launch_file"),
                "bridge_node_name": LaunchConfiguration("bridge_node_name"),
                "viz_node_name": LaunchConfiguration("viz_node_name"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_viz_mode_manager")),
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
            web_root
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
        DeclareLaunchArgument("style_preset", default_value="cool_steel"),
        DeclareLaunchArgument("frame_id", default_value="x_taxel_viz"),
        DeclareLaunchArgument("grid_parent_frame", default_value="world"),
        DeclareLaunchArgument("legacy_out_topic", default_value="/x_taxel_2f/markers"),
        DeclareLaunchArgument("fixed_frame", default_value="world"),
        DeclareLaunchArgument("left_module_index", default_value="0"),
        DeclareLaunchArgument("right_module_index", default_value="1"),
        DeclareLaunchArgument("grid_rows", default_value="4"),
        DeclareLaunchArgument("grid_cols", default_value="6"),
        DeclareLaunchArgument("cell_size", default_value="0.015"),
        DeclareLaunchArgument("module_gap", default_value="0.04"),
        DeclareLaunchArgument("row_flip_right", default_value="true"),
        DeclareLaunchArgument("col_flip_right", default_value="true"),
        DeclareLaunchArgument("grid_left_force_x_sign", default_value="1.0"),
        DeclareLaunchArgument("grid_left_force_y_sign", default_value="-1.0"),
        DeclareLaunchArgument("grid_right_force_x_sign", default_value="-1.0"),
        DeclareLaunchArgument("grid_right_force_y_sign", default_value="-1.0"),
        DeclareLaunchArgument("urdf_left_force_x_sign", default_value="1.0"),
        DeclareLaunchArgument("urdf_left_force_y_sign", default_value="1.0"),
        DeclareLaunchArgument("urdf_right_force_x_sign", default_value="1.0"),
        DeclareLaunchArgument("urdf_right_force_y_sign", default_value="1.0"),
        DeclareLaunchArgument("max_publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("publisher_transient_local", default_value="false"),
        DeclareLaunchArgument("marker_stamp_mode", default_value="now"),
        DeclareLaunchArgument("circle_marker_type", default_value="sphere"),
        DeclareLaunchArgument("emit_urdf_points", default_value="false"),
        DeclareLaunchArgument("freeze_urdf_positions", default_value="true"),
        DeclareLaunchArgument("enable_viz_mode_manager", default_value="true"),
        DeclareLaunchArgument("managed_launch_package", default_value="ur5e_x2f_140_config2"),
        DeclareLaunchArgument("managed_launch_file", default_value="xela_viz_safe.launch.py"),
        DeclareLaunchArgument("bridge_node_name", default_value="/xela_taxel_web_bridge_cpp"),
        DeclareLaunchArgument("viz_node_name", default_value="/xela_taxel_viz_2f"),
        DeclareLaunchArgument("bridge_tf_topic", default_value="/tf"),
        DeclareLaunchArgument("bridge_tf_static_topic", default_value="/tf_static"),
        DeclareLaunchArgument("enable_web_server", default_value="true"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8765"),
        DeclareLaunchArgument("enable_sidecar_rosbridge", default_value="true"),
        DeclareLaunchArgument("sidecar_rosbridge_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("sidecar_rosbridge_port", default_value="9090"),
        sidecar_rosbridge,
        bridge_node,
        mode_manager_node,
        web_server,
    ])
