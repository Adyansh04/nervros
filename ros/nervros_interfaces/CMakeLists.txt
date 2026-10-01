cmake_minimum_required(VERSION 3.16)
project(nervros_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteMission.action"
  "msg/Heartbeat.msg"
  "msg/NodeEvent.msg"
  "msg/RobotState.msg"
  "msg/StepPreview.msg"
  "srv/GetCatalog.srv"
  "srv/PreviewMission.srv"
  "srv/StopAll.srv"
  "srv/Teleop.srv"
  "srv/ValidateMission.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs
)

ament_package()
