cmake_minimum_required(VERSION 3.5)
project(kaqu_gazebo_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kaqu_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ros2_control REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executables
add_executable(kaqu_gazebo_joint_ctrl_node src/kaqu_gazebo_joint_controller.cpp)
ament_target_dependencies(kaqu_gazebo_joint_ctrl_node
  kaqu_msgs
  trajectory_msgs
  nav_msgs
  gazebo_ros
  ros2_control
  std_msgs
  sensor_msgs
)

# Install executables
install(TARGETS
  kaqu_gazebo_joint_ctrl_node  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)
# Install resources
install(DIRECTORY
  config
  description
  rviz
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()