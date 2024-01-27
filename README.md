# CAVPsim
This repo contains source codes of CAVPsim a simulator for Connected Autonomous Vehicles Planning research scope <a id="1">[1]</a>. CAVPsim contains of three main component of vehicle motion model, communication application layer model and computation model. This repo also contains demo launch files and script to execute RAD-VEGA a Resource Aware and Distributed implementation of Vector Evaluated Genetic Algorithm as Multi-Objective Optimization Probmel (MOOP) solver which is modeled as speed profile optimizer.
This repo uses some of autoware.universe open source stack.  

## Build instructions
The minimume requirements to build the CAVPsim and RAD-VEGA implemenation:
- Ubuntu 18.04 (ROS melodic) or 20.04 (noetic)
- 8 Gb RAM (To execute 6 agent scenario for RAD-VEGA)
- 2 Gb free storage (for source codes, binaries)
```
$ cd CAVPsim
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release 
$ catkin config --extend /opt/ros/$ROS_DISTRO/  
$ catkin build -c -s
```
## Run demo scenarios for RAD-VEGA evaluation
After sucessful build you can execute RAD-VEGA a solver implemented in CAVPsim to solve two set of sample problems. The first set is to run RAD-VEGA to solve ZDT1 MOOP test problem. The second set is distributed decision making problem for number of CAVs in a real HD map. To run ZDT1 test problem: 
```
$ cd CAVPsim/src/launch_scripts/
$ sudo chmod +x ZDT_RAD_VEGA_SOVLER.sh
$ ./ZDT_RAD_VEGA_SOLVER.sh
```
For planning scenario problem:
```
$ cd CAVPsim/src/launch_scripts/
$ sudo chmod +x CDDP_RAD_VEGA_SOVLER.sh
$ ./CDDP_RAD_VEGA_SOLVER.sh
```
The launcher script will ask for number of CAVs.

## References
<a id="1">[1]</a> 
Ghahremaninejad, R., & Bilgen, S. (2022). Introducing a Novel ROS-based Cooperative Autonomous Vehicles Planning Simulation Framework, CAVPsim. In VEHITS (pp. 208-215).