cmake_minimum_required(VERSION 2.8.3)
project(rover_ctrl_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

include_directories( LibRover )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -I../LibRover")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L../LibRover")
add_library(Rover LibRover/BogieScanner.cc LibRover/CANopenHelper.cc LibRover/CANopenMaster.cc LibRover/Drive.cc LibRover/Track.cc LibRover/libRover.cc)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rover_ctrl_interface
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(simple_motor_test src/simple_motor_test.cpp)
target_link_libraries(simple_motor_test ${catkin_LIBRARIES} Rover)

add_executable(rover_interface src/rover_interface.cpp)
target_link_libraries(rover_interface ${catkin_LIBRARIES} Rover)
