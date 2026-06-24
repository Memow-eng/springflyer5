cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs cv_bridge message_filters pcl_conversions
 DEPENDS OpenCV PCL
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( plan_env 
    src/grid_map.cpp 
    src/raycast.cpp
    src/obj_predictor.cpp 
    )
target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )  

add_executable(obj_generator
    src/obj_generator.cpp 
)
target_link_libraries(obj_generator 
    ${catkin_LIBRARIES}
    )
