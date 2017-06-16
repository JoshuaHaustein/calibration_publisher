cmake_minimum_required(VERSION 2.8.3)
project(calibration_publisher)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs tf cmake_modules)
FIND_PACKAGE(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES 
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf cmake_modules
  DEPENDS 
)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(calibration_publisher src/calibration_publisher.cpp)
target_link_libraries(calibration_publisher yaml-cpp ${catkin_LIBRARIES})
