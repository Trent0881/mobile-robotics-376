cmake_minimum_required(VERSION 2.8.3)
project(wps)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(example_ros_service src/example_ros_service.cpp)
cs_add_executable(example_ros_client src/example_ros_client.cpp)
cs_add_executable(path_service src/path_service.cpp)
cs_add_executable(mps2 src/mps2.cpp)
cs_add_executable(mpc src/mpc.cpp)
cs_install()
cs_export()
    
