cmake_minimum_required(VERSION 2.8.3)
project(solver_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  cav_vehicle_model_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  solutionHolder.msg
  solutionHolderArr.msg
  finalSolution.msg
  finalSolutionArr.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  cav_vehicle_model_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  cav_vehicle_model_msgs
)
