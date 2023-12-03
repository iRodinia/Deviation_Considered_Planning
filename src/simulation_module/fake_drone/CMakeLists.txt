cmake_minimum_required(VERSION 3.5)
project(fake_drone)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf2_ros
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (fake_drone_node src/fake_drone.cpp)
target_link_libraries(fake_drone_node
  ${catkin_LIBRARIES}
)
