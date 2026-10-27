cmake_minimum_required(VERSION 3.8)
project(fleet_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/Move.msg"
  "msg/Control.msg"
  "msg/Display.msg"
  "msg/RobotState.msg"
  "msg/RobotEvent.msg"
  DEPENDENCIES builtin_interfaces
)

# 노드 그래프 · 맵 · 정밀 정차 설정을 share에 설치 (C++에서 get_package_share_directory("fleet_msgs") + "/docs/...")
install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
