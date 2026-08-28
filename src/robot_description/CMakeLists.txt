cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)

# 显式安装 aider 模型资源，确保 package://robot_description/... 可解析
install(
  DIRECTORY urdf/aider meshes/aider
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
