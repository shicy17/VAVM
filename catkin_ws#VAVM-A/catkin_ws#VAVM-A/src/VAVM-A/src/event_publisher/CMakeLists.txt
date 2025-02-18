cmake_minimum_required(VERSION 2.8.3)
project(event_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  cv_bridge
  sensor_msgs
  dvs_msgs
  eigen_conversions
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(event_publisher src/main.cpp)
target_link_libraries(event_publisher
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)