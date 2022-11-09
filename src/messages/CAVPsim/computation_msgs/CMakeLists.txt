cmake_minimum_required(VERSION 2.8.3)
project(computation_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  solver_msgs
  # communication_msgs
  autoware_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  status.msg
  VEGA_stat.msg
  RAD_VEGA_POPULATION.msg
  RAD_VEGA_SOLUTION.msg
  NET_MEMBER.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  solver_msgs
  autoware_msgs
  # communication_msgs

)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  solver_msgs
  # communication_msgs
  autoware_msgs
)
