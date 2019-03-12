cmake_minimum_required(VERSION 3.1.0)
project(darknet_actions)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  actionlib
  darknet_ros_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

set (CMAKE_CXX_STANDARD 11)

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   obj_detection.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
 )
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES darknet_actions
   CATKIN_DEPENDS roscpp rospy std_msgs actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(obj_detection_server src/obj_detection_server.cpp)
add_executable(obj_detection_client src/obj_detection_client.cpp)

target_link_libraries(
    obj_detection_server
    ${catkin_LIBRARIES}
)
target_link_libraries(
    obj_detection_client
    ${catkin_LIBRARIES}
)
add_dependencies(
    obj_detection_server
    obj_detection_client
    ${darknet_actions_EXPORTED_TARGETS}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)


