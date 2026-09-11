cmake_minimum_required(VERSION 3.8)
project(xela_taxel_viz_core)

find_package(ament_cmake REQUIRED)

# Static web resources only (JS core/render modules + vendor libs). No ROS nodes.
install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

ament_package()
