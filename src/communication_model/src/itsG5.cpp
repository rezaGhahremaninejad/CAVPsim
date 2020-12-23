#include "ros/ros.h"
#include "model_msgs/VehicleModelInput.h"
#include "model_msgs/VehicleModelOutput.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <communication_msgs/status.h>

#include <ctime>
#include <cmath>


int seq;
double BAND_WIDTH, MESSAGE_RATE;

model_msgs::VehicleModelOutput vehicle_output_tx;
model_msgs::VehicleModelInput vehicle_input_tx;
bool vehicle_output_tx_ready = false;
bool vehicle_input_tx_ready = false;
ros::Time vehicle_input_rx_timeStamp;
ros::Time vehicle_output_rx_timeStamp;

using namespace std;

void input_rxCallback(const model_msgs::VehicleModelInput msg)
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

void output_rxCallback(const model_msgs::VehicleModelOutput msg)
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
    ros::init(argc, argv, "itsG5");
    ros::NodeHandle n;
    n.param<double>("BAND_WIDTH", BAND_WIDTH, 6.0); //Mb/s
    n.param<double>("MESSAGE_RATE", MESSAGE_RATE, 0.1);  //seconds
    ros::Publisher vehicle_output_tx_pub = n.advertise<model_msgs::VehicleModelOutput>("output_tx", 1000);
    ros::Publisher vehicle_input_tx_pub = n.advertise<model_msgs::VehicleModelInput>("input_tx", 1000);
    ros::Publisher status_pub = n.advertise<communication_msgs::status>("status", 1000);
    ros::Subscriber vehicle_input_rx_sub = n.subscribe("input_rx", 1000, input_rxCallback);
    ros::Subscriber vehicle_ouput_rx_sub = n.subscribe("output", 1000, output_rxCallback);

    ros::Rate loop_rate(30);
    bool newStatus;
    //vehicle_output_tx_ready = true;
    while (ros::ok())
    {
        ros::spinOnce();
        // TODO
        communication_msgs::status com_status;
        newStatus = false;
        if (vehicle_output_tx_ready) {
            int s = sizeof(vehicle_output_tx);
            float p = exp(-2*s/(1000000*BAND_WIDTH*MESSAGE_RATE));
            float my_rnd = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

            ROS_INFO("----------out--p, rand: %f, %f", p, rand);
            if (p>= my_rnd) {
                vehicle_output_tx.header.stamp = ros::Time::now();
                vehicle_output_tx_pub.publish(vehicle_output_tx);
                vehicle_output_tx_ready = false;
                com_status.vehicle_output_rx_timeStamp = vehicle_output_rx_timeStamp;
                com_status.vehicle_output_tx_timeStamp = vehicle_output_tx.header.stamp;
                com_status.total_delay_sec = com_status.vehicle_output_tx_timeStamp.toNSec() - com_status.vehicle_output_rx_timeStamp.toNSec();
                com_status.missed_msgs =  1.0*vehicle_output_tx.header.seq -  1.0*seq;
                seq = seq + 1;
                newStatus = true;
            }
        }

        if (vehicle_input_tx_ready) {
            int s = sizeof(vehicle_output_tx);
            float p = exp(-2*s/(BAND_WIDTH*MESSAGE_RATE));
            ROS_INFO("----------in--p: %f", p);
            if (p<= static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) {
                vehicle_input_tx.header.stamp = ros::Time::now();
                vehicle_input_tx_pub.publish(vehicle_input_tx);
                vehicle_input_tx_ready = false;
                com_status.vehicle_input_rx_timeStamp = vehicle_input_rx_timeStamp;
                com_status.vehicle_input_tx_timeStamp = vehicle_input_tx.header.stamp;
                com_status.total_delay_sec = com_status.vehicle_input_tx_timeStamp.toNSec() - com_status.vehicle_input_rx_timeStamp.toNSec();
                com_status.missed_msgs = 1.0*vehicle_input_tx.header.seq - 1.0*seq;
                seq = seq + 1;
                newStatus = true;
            }
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