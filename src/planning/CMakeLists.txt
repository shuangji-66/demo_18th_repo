cmake_minimum_required(VERSION 2.8.3)
project(planning_node)

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
  )

  find_package(Eigen3 QUIET)
  if (NOT EIGEN3_FOUND)
      find_package(PkgConfig REQUIRED)
      pkg_check_modules(EIGEN3 REQUIRED eigen3)
      set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
  endif ()
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
  /usr/include/eigen3
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/planning_node_handle.cpp
  src/planning_node.cpp
  src/main.cpp
  src/planner1.cpp
  src/cubic_spline.cpp
  src/planner_base.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
