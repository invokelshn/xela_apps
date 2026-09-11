cmake_minimum_required(VERSION 3.8)
project(xela_taxel_operator_dg5f)

find_package(ament_cmake REQUIRED)

# No ROS nodes/executables of its own -- just launch (starts a dedicated rosbridge_server
# instance + a static web server script) and static web assets (operator-only page that
# loads xela_taxel_viz_core + xela_atag_taxel_viewer JS).
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

ament_package()
