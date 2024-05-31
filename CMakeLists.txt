cmake_minimum_required(VERSION 3.5)
project(turtlebot3_odom)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_executable(odometry src/odometry.cpp)

ament_package()
