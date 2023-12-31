cmake_minimum_required(VERSION 3.0.2)
project(serial_port)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(FILES Imu_0x91_msg.msg Imu_0x62_msg.msg Imu_data_package.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)


################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_port
   CATKIN_DEPENDS roscpp std_msgs rospy serial message_runtime message_generation
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
)

add_executable(serial_imu src/serial_imu.cpp src/ch_serial.c)
target_link_libraries(serial_imu ${catkin_LIBRARIES})

add_executable(sub_spec src/sub_spec.cpp)
add_dependencies(sub_spec ${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(sub_spec ${catkin_LIBRARIES})


#############
## Install ##
#############


#############
## Testing ##
#############


