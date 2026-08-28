cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/RobotState.msg
  msg/ControlGoal.msg
  msg/JointCommand.msg
  msg/GaitState.msg
  msg/IMUArray.msg
  msg/ExoInput.msg
  msg/InferenceRequest.msg
  msg/InferenceResult.msg
  msg/SkillCommand.msg
  srv/SetMode.srv
  srv/Calibrate.srv
  srv/GetRobotState.srv
  srv/LoadModel.srv
  action/Navigate.action
  action/Grasp.action
  action/Trajectory.action
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
