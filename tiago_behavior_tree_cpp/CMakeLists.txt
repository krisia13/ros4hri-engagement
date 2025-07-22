cmake_minimum_required(VERSION 3.8)
project(tiago_behavior_tree_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create main executable
add_executable(${PROJECT_NAME}_node
  src/tiago_bt_node.cpp
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  behaviortree_cpp_v3
  ament_index_cpp
  nav2_msgs 
  tf2_ros   
  tf2_geometry_msgs  
)

# Install
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  trees
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()