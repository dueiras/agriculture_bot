cmake_minimum_required(VERSION 3.5)
project(navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  params
  maps
  rviz2
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS 
  navigation/waypoint_follow.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
