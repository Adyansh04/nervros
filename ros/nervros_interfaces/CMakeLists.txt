cmake_minimum_required(VERSION 3.16)
project(nervros_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteMission.action"
  "msg/NodeEvent.msg"
  "msg/RobotState.msg"
  "srv/GetCatalog.srv"
  "srv/StopAll.srv"
  "srv/ValidateMission.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
