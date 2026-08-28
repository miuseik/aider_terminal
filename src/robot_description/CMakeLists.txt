cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

# 安装全部资源：urdf(含 xacro) / meshes / rviz / launch
# 标准 ROS2 做法：整目录安装到 share/robot_description/，
# 使 package://robot_description/... 可解析
install(
  DIRECTORY urdf meshes rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
