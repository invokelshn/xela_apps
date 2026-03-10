cmake_minimum_required(VERSION 3.8)
project(xela_taxel_viz_2f)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xela_taxel_msgs REQUIRED)

add_executable(xela_taxel_viz_2f_node
  src/xela_taxel_viz_2f.cpp
)

ament_target_dependencies(xela_taxel_viz_2f_node
  rclcpp
  std_msgs
  std_srvs
  visualization_msgs
  geometry_msgs
  xela_taxel_msgs
)

install(TARGETS xela_taxel_viz_2f_node
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

ament_package()
