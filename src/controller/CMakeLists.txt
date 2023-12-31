cmake_minimum_required(VERSION 2.8.3)
project(controller)

add_compile_options(-std=c++11)

set(PROJECT_DEPS
  roscpp
  std_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  ucar_common_msgs
  sensor_msgs
  tf
  ackermann_msgs
  )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS 
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/controller_handle.cpp
  src/controller.cpp
  src/main.cpp
  src/pid_controller.cpp
  src/solver_base.cpp
  src/stanley_solver.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
