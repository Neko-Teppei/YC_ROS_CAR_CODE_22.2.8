cmake_minimum_required(VERSION 2.8.3)
project(ulstrasonic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  geometry_msgs 
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ulstramsg.msg
 )


## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    std_msgs
  )


catkin_package(
  CATKIN_DEPENDS 
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES mmwave_pkg
#  CATKIN_DEPENDS roscpp rospy serial std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
add_executable(ulstrasonic src/ulstrasonic.cpp)
target_link_libraries(ulstrasonic ${catkin_LIBRARIES})
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
