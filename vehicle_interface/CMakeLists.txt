cmake_minimum_required(VERSION 2.8.3)
project(vehicle_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

add_service_files(
  FILES
  sonarScan.srv
)

generate_messages(
  DEPENDENCIES
  roscpp
  std_msgs
  sensor_msgs
)

find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vehicle_interface
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs
  DEPENDS boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(interface src/interface.cpp src/arduino_serial.cpp src/arduino_interface.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(interface ${catkin_LIBRARIES})

