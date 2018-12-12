cmake_minimum_required(VERSION 2.8.3)
project(waypoint_navigation)


######################################
## Find catkin macros and libraries ##
######################################

find_package(catkin REQUIRED COMPONENTS
  apriltags
  apriltags_ros
  roscpp
  rospy
  std_msgs
  message_generation
  mavros_msgs
)



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Num.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   AddTwoInts.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)




###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  LIBRARIES apriltag_detector
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
### Model: add_executable(node_namefile src/node_namefile.cpp)
add_executable(offb_node src/offb_node.cpp)


## Add cmake target dependencies of the executable
### Model: add_dependencies(node_namefile package_name_generate_messages_cpp)
add_dependencies(offb_node waypoint_navigation_generate_messages_cpp)


## Specify libraries to link a library or executable target against
### Model: # target_link_libraries(node_namefile ${catkin_LIBRARIES})
target_link_libraries(offb_node ${catkin_LIBRARIES})

