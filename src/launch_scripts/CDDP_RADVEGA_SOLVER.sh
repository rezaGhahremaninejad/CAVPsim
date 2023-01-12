#!/bin/bash
echo "Set number of CAVs: "
read CAV_NUMBER
rosparam set use_sim_time true & 
roslaunch communication_model default.launch &
roslaunch ../launch_files/common.launch &

for i in $(seq 1 $CAV_NUMBER)
do
  # echo "Launch CAV $i"
  # sleep 0.1
  roslaunch ../launch_files/PLANNING_RADVEGA_SINGLE.launch vehicle_namespace:="v_$i" &
done