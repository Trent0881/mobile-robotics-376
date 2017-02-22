cmake_minimum_required(VERSION 2.8.3)
project(pas)

find_package(catkin_simple REQUIRED)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# Executables
cs_add_executable(ac src/ac.cpp)
cs_add_executable(as src/as.cpp)

cs_install()
cs_export()
    
