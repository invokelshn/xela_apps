cmake_minimum_required(VERSION 3.8)
project(xela_taxel_viz_ahv4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xela_taxel_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(xela_taxel_viz_ahv4_node
  src/xela_taxel_viz_ahv4.cpp
)

add_executable(xela_taxel_viz_ahv4_demo_node
  src/xela_taxel_viz_ahv4_demo.cpp
)

ament_target_dependencies(xela_taxel_viz_ahv4_node
  rclcpp
  std_msgs
  visualization_msgs
  geometry_msgs
  xela_taxel_msgs
)

ament_target_dependencies(xela_taxel_viz_ahv4_demo_node
  rclcpp
  xela_taxel_msgs
)

target_link_libraries(xela_taxel_viz_ahv4_node
  yaml-cpp
)

target_link_libraries(xela_taxel_viz_ahv4_demo_node
  yaml-cpp
)

install(TARGETS xela_taxel_viz_ahv4_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS xela_taxel_viz_ahv4_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

install(PROGRAMS
  scripts/csv_to_pattern_yaml.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
