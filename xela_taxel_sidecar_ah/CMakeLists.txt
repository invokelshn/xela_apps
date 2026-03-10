cmake_minimum_required(VERSION 3.8)
project(xela_taxel_sidecar_ah)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xela_taxel_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(xela_taxel_web_bridge_node
  src/xela_taxel_web_bridge_node.cpp
)
ament_target_dependencies(xela_taxel_web_bridge_node
  rclcpp
  rcl_interfaces
  std_msgs
  tf2
  tf2_ros
  xela_taxel_msgs
)
target_link_libraries(xela_taxel_web_bridge_node yaml-cpp)

add_executable(xela_viz_mode_manager_node
  src/xela_viz_mode_manager_node.cpp
)
ament_target_dependencies(xela_viz_mode_manager_node
  rclcpp
  rcl_interfaces
  std_srvs
)

install(TARGETS
  xela_taxel_web_bridge_node
  xela_viz_mode_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

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
