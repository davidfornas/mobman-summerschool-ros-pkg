cmake_minimum_required(VERSION 2.8.3)
project(vehicle_perception)

find_package(OpenCV REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  image_transport
  std_srvs
)

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
#  LIBRARIES vehicle_perception
  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs roscpp sensor_msgs image_transport std_srvs
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(surf_detection_example src/surf_detection_example.cpp src/surf_detector.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(surf_detection_example ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
