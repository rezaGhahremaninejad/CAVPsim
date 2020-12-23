cmake_minimum_required(VERSION 2.8.3)
project(computation_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  solver_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  status.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  solver_msgs

)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  solver_msgs

)
