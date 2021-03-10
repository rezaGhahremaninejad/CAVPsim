#include "ros/ros.h"
#include "vehicle_model_msgs/VehicleModelInput.h"
#include "vehicle_model_msgs/VehicleModelOutput.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <communication_msgs/status.h>

#include <ctime>
#include <cmath>

double delay_sec;

int seq;

vehicle_model_msgs::VehicleModelOutput vehicle_output_tx;
vehicle_model_msgs::VehicleModelInput vehicle_input_tx;
bool vehicle_output_tx_ready = false;
bool vehicle_input_tx_ready = false;
ros::Time vehicle_input_rx_timeStamp;
ros::Time vehicle_output_rx_timeStamp;

using namespace std;

void input_rxCallback(const vehicle_model_msgs::VehicleModelInput msg)
{
    if (vehicle_input_tx_ready)
    {
    }
    else
    {
        vehicle_input_rx_timeStamp = ros::Time::now();
        vehicle_input_tx = msg;
        vehicle_input_tx_ready = true;
    }

    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void output_rxCallback(const vehicle_model_msgs::VehicleModelOutput msg)
{
    if (vehicle_output_tx_ready)
    {
    }
    else
    {
        vehicle_output_rx_timeStamp = ros::Time::now();
        vehicle_output_tx = msg;
        vehicle_output_tx_ready = true;
    }

    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "backbone_model");
    ros::NodeHandle n;
    n.param<double>("delay_sec", delay_sec, 0.0005);
    ros::Publisher vehicle_output_tx_pub = n.advertise<vehicle_model_msgs::VehicleModelOutput>("output_tx", 1000);
    ros::Publisher vehicle_input_tx_pub = n.advertise<vehicle_model_msgs::VehicleModelInput>("input_tx", 1000);
    ros::Publisher status_pub = n.advertise<communication_msgs::status>("status", 1000);
    ros::Subscriber vehicle_input_rx_sub = n.subscribe("input_rx", 1000, input_rxCallback);
    ros::Subscriber vehicle_ouput_rx_sub = n.subscribe("output_rx", 1000, output_rxCallback);

    ros::Rate loop_rate(1000);

    while (ros::ok())
    {
        ros::spinOnce();
        // TODO
        bool newStatus = false;
        communication_msgs::status com_status;
        ros::Duration d(delay_sec);
        if (ros::Time::now() >= vehicle_output_rx_timeStamp + d && vehicle_output_tx_ready)
        {
            vehicle_output_tx.header.stamp = ros::Time::now();
            vehicle_output_tx_pub.publish(vehicle_output_tx);
            vehicle_output_tx_ready = false;
            com_status.vehicle_output_rx_timeStamp = vehicle_output_rx_timeStamp;
            com_status.vehicle_output_tx_timeStamp = vehicle_output_tx.header.stamp;
            com_status.total_delay_sec = com_status.vehicle_output_tx_timeStamp.toSec() - com_status.vehicle_output_rx_timeStamp.toSec();
            com_status.delay_sec = delay_sec;
            com_status.missed_msgs =  1.0*vehicle_output_tx.header.seq -  1.0*seq;
            seq = seq + 1;
            newStatus = true;
        }

        if (ros::Time::now() >= vehicle_input_rx_timeStamp + d && vehicle_input_tx_ready)
        {
            vehicle_input_tx.header.stamp = ros::Time::now();
            vehicle_input_tx_pub.publish(vehicle_input_tx);
            vehicle_input_tx_ready = false;
            com_status.vehicle_input_rx_timeStamp = vehicle_input_rx_timeStamp;
            com_status.vehicle_input_tx_timeStamp = vehicle_input_tx.header.stamp;
            com_status.delay_sec = delay_sec;
            com_status.total_delay_sec = com_status.vehicle_input_tx_timeStamp.toSec() - com_status.vehicle_input_rx_timeStamp.toSec();
            com_status.missed_msgs = 1.0*vehicle_input_tx.header.seq - 1.0*seq;
            seq = seq + 1;
            newStatus = true;
        }

        if (newStatus)
        {
            com_status.header.stamp = ros::Time::now();
            com_status.header.seq = seq;
            status_pub.publish(com_status);
            newStatus = false;
        }

        //loop_rate.sleep();
    }
    return 0;
}