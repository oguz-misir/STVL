cmake_minimum_required(VERSION 3.8)
project(negative_obstacle_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
