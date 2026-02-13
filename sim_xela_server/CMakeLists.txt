cmake_minimum_required(VERSION 3.8)
project(sim_xela_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

add_executable(sim_xela_server_node
  src/sim_xela_server.cpp
)

target_include_directories(sim_xela_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
)

target_link_libraries(sim_xela_server_node
  Boost::system
  Boost::thread
)

ament_target_dependencies(sim_xela_server_node
  rclcpp
  ament_index_cpp
)

install(TARGETS sim_xela_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
