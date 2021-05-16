#include "ros/ros.h"
#include "cav_vehicle_model_msgs/VehicleModelOutput.h"
#include "communication_msgs/ComMessage.h"
#include "computation_msgs/status.h"
#include "autoware_msgs/LaneArray.h"
#include "cooperative_msgs/status.h"
#include <cmath>

communication_msgs::ComMessage _tx_com;

void vehicleOutCallback(const cav_vehicle_model_msgs::VehicleModelOutput::ConstPtr &msg)
{
    _tx_com.header.stamp = ros::Time::now();
    _tx_com.cav_vehicle_model_out = *msg;
}

void compStatusCallback(const computation_msgs::status::ConstPtr &msg)
{
    _tx_com.header.stamp = ros::Time::now();
    _tx_com.computation_status = *msg;
}

void egoPathCAllBack(const autoware_msgs::LaneArray::ConstPtr & msg) {
    _tx_com.header.stamp = ros::Time::now();
    _tx_com.ego_path = *msg;
}

void coopStatusCallBack(const cooperative_msgs::status::ConstPtr & msg) {
    _tx_com.header.stamp = ros::Time::now();
    _tx_com.ego_status = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gate_node");
    ros::NodeHandle n;
    // n.param<float>("model/L", L, 2.7);                             //wheelbase
    ros::Publisher tx_pub = n.advertise<communication_msgs::ComMessage>("tx_com", 1000);
    ros::Subscriber vehicle_output_sub = n.subscribe("/cav_vehicle_model/output", 1000, vehicleOutCallback);
    ros::Subscriber computation_sub = n.subscribe("/computation/status", 1000, compStatusCallback);
    ros::Subscriber ego_path_sub = n.subscribe("/lane_waypoints_array", 1000, egoPathCAllBack);
    ros::Subscriber cooperation_status_sub = n.subscribe("/computation/cooperation_status", 1000, coopStatusCallBack);
    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        ros::spinOnce();
        _tx_com.header.stamp = ros::Time::now();
        tx_pub.publish(_tx_com);
        loop_rate.sleep();
    }
    return 0;
}