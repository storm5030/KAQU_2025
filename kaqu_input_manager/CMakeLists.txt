cmake_minimum_required(VERSION 3.5)
project(kaqu_input_manager)

# 기본 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(kaqu_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(kaqu_gamepad_node src/kaqu_teleop_joy.cpp)
ament_target_dependencies(kaqu_gamepad_node 
  rclcpp 
  sensor_msgs 
  std_msgs 
  kaqu_msgs
  geometry_msgs
  trajectory_msgs
)

# 실행 파일 설치 설정
install(TARGETS
  kaqu_gamepad_node
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/joystick_gui_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 런치 디렉토리 설치 설정
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
