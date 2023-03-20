cmake_minimum_required(VERSION 3.8)
project(kdl_inverse_dynamics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lbr_fri_msgs REQUIRED)
find_package(kdl_parser REQUIRED)

set(kdl_inverse_dynamics_node_dependencies
  kdl_parser
  rclcpp
  lbr_fri_msgs
)

add_executable(kdl_inverse_dynamics_node
  kdl_inverse_dynamics_node.cpp
)

ament_target_dependencies(kdl_inverse_dynamics_node
  ${kdl_inverse_dynamics_node_dependencies}
)

install(
  TARGETS kdl_inverse_dynamics_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
