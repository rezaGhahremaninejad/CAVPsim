#include "ros/ros.h"
#include <ctime>
#include <cmath>
#include <iostream>
#include <cstdio>
#include "rosgraph_msgs/Clock.h"
#include <chrono>
// #include "../include/Timer.h"
using namespace std;
clock_t _time;
// void output_rxCallback(const cav_vehicle_model_msgs::VehicleModelOutput msg)
// {
//     if (vehicle_output_tx_ready)
//     {
//     }
//     else
//     {
//         vehicle_output_rx_timeStamp = ros::Time::now();
//         vehicle_output_tx = msg;
//         vehicle_output_tx_ready = true;
//     }

//     //ROS_INFO("I heard: [%s]", msg->data.c_str());
// }
using namespace std::chrono;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cavpsim_clock_node");
    ros::NodeHandle n;
    ros::Publisher time_pub = n.advertise<rosgraph_msgs::Clock>("clock", 1000);

    rosgraph_msgs::Clock _clock;
    
    ros::Rate loop_rate(1000);

    high_resolution_clock::time_point t1;
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
   
    while (ros::ok())
    {
        t1 = high_resolution_clock::now();
        duration<double> time_span = duration_cast<duration<double>>(t1 - t2);
        _clock.clock.sec = int(time_span.count());
        _clock.clock.nsec = 1000000000*(time_span.count() - _clock.clock.sec);
        time_pub.publish(_clock);

        // ros::spinOnce();
        // ros::spin();
        // loop_rate.sleep();
    }
    return 0;
    cout << "------------breaking" << endl;
}