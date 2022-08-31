#include "ros/ros.h"
#include <ctime>
#include <cmath>
#include <iostream>
#include <cstdio>
#include <chrono>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/Lane.h>
// #include "../include/Timer.h"
int POP_SIZE = 15;
bool INIT = false;
using namespace std;
clock_t _time;
using namespace std::chrono;

int RAND(const int a, const int b) 
{return b + ( std::rand() % ( a - b + 1));}
ros::Publisher laneArr_pub;
autoware_msgs::LaneArray generateInitialPopForZDT1() {
    autoware_msgs::LaneArray _result;
    autoware_msgs::Lane _lane;
    for (int i = 0; i < POP_SIZE; i++){
        autoware_msgs::Waypoint _thisWayPoint;
        _thisWayPoint.twist.twist.linear.x = 0.01*(RAND(200, 100) - 100);
        _lane.waypoints.push_back(_thisWayPoint);
    }
    _result.lanes.push_back(_lane);
    return _result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zdt_init");
    ros::NodeHandle n;
    laneArr_pub = n.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1000);
    
    ros::Rate loop_rate(10);
    autoware_msgs::LaneArray _this_lane_arr;
    _this_lane_arr = generateInitialPopForZDT1();

    while (ros::ok())
    {
        // if (laneArr_pub.getNumSubscribers() >= 1) {
        // std::cout << "sub check...." << std::endl;
        // ros::spin();
        if (!INIT){
            // std::cout << "INIT check...." << std::endl;
            laneArr_pub.publish(_this_lane_arr);
            // INIT = true;
        }
        // }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}