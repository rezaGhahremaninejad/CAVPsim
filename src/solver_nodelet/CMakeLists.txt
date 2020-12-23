cmake_minimum_required(VERSION 2.8.3)
project(solver_nodelet)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs solver_msgs model_msgs communication_msgs computation_msgs tf)

## Setup include directories
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES solver_nodelet
  CATKIN_DEPENDS nodelet roscpp std_msgs solver_msgs model_msgs communication_msgs tf
)

## Create the nodelet tutorial library
add_library(solver_nodelet src/plus.cpp)
target_link_libraries(solver_nodelet ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(solver_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS solver_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES solver_nodelet.xml plus.launch plus_default.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
