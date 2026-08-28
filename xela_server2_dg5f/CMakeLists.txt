cmake_minimum_required(VERSION 3.8)
project(xela_server2_dg5f)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xela_taxel_msgs REQUIRED)
find_package(xela_server_ros2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

add_library(xela_server2_dg5f_lib
  src/dg5f_parser.cpp
  src/dg5f_joint_map.cpp
)

target_include_directories(xela_server2_dg5f_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
)

target_link_libraries(xela_server2_dg5f_lib
  yaml-cpp
)

ament_target_dependencies(xela_server2_dg5f_lib
  rclcpp
  xela_taxel_msgs
  xela_server_ros2
)

add_executable(xela_server2_dg5f_node
  src/dg5f_node.cpp
)

target_link_libraries(xela_server2_dg5f_node
  xela_server2_dg5f_lib
  Boost::system
  Boost::thread
)

ament_target_dependencies(xela_server2_dg5f_node
  rclcpp
  std_msgs
  xela_taxel_msgs
  xela_server_ros2
  ament_index_cpp
)

install(TARGETS xela_server2_dg5f_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
