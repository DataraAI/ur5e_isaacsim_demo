cmake_minimum_required(VERSION 3.22)
project(ur5e_isaac_bridge)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
