#include "ros/ros.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include "std_msgs/String.h"
#include "computation_msgs/VEGA.h"
#include "computation_msgs/status.h"
#include "autoware_msgs/LaneArray.h"
#include "cooperative_msgs/status.h"

void rxCallback(const communication_msgs::ComMessage::ConstPtr &msg)
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "model");
    ros::NodeHandle n;

    n.param<double>("/compute_model/batch_size", batch_size, 10000);
    n.param<double>("/compute_model/CAV_t_available", CAV_t_available, 0.01);
    n.param<double>("/compute_model/app_flo", app_flo, 10000);
    n.param<float>("/compute_model/MIN_POINTS_DIST", MIN_POINTS_DIST, 1);
    n.param<float>("/compute_model/SAFE_DISTANCE", SAFE_DISTANCE, 6);

    ros::Publisher status_pub = n.advertise<computation_msgs::VEGA>("/computation/status", 1000);
    ros::Publisher cooperation_status_pub = n.advertise<cooperative_msgs::status>("/computation/cooperation_status", 1000);
    updated_lane_pub = n.advertise<autoware_msgs::LaneArray>("/computation/updated_lane_waypoints_array", 1000);
    ros::Subscriber computation_sub = n.subscribe("/rx_com", 1000, rxCallback);
    ros::Subscriber ego_path_sub = n.subscribe("/lane_waypoints_array", 1000, egoPathCallback);

    srand(static_cast<unsigned>(time(0)));

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        //ros::spin();
        loop_rate.sleep();
    }
    return 0;
}