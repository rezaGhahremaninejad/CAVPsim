#include "ros/ros.h"
#include "cav_vehicle_model_msgs/VehicleModelInput.h"
#include "cav_vehicle_model_msgs/VehicleModelOutput.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <communication_msgs/status.h>
#include <communication_msgs/BristolRx.h>
#include <communication_msgs/BristolTx.h>
#include <ctime>
#include <cmath>

int seq;
double BAND_WIDTH, MESSAGE_RATE;

communication_msgs::BristolRx _v01_Rx_sim;
// bool _v01_Rx_sim_ready = false;
bool _tx_ready = false;
//ros::Time _v01_Rx_sim_timeStamp;

using namespace std;

void input_rxCallback(const communication_msgs::BristolTx msg)
{
    _v01_Rx_sim.RxMAC = "4c:5e:0c:84:35:f6";
    _v01_Rx_sim.header = msg.header;
    _v01_Rx_sim.SeqNum = msg.SeqNum;
    _v01_Rx_sim.GpsLon = msg.GpsLon;
    _v01_Rx_sim.GpsLat = msg.GpsLat;
    _v01_Rx_sim.CamLon = msg.CamLon;
    _v01_Rx_sim.CamLat = msg.CamLat;
    _v01_Rx_sim.Timestamp = msg.Timestamp;

    int s = sizeof(_v01_Rx_sim);
    float p = exp(-2 * s / (BAND_WIDTH * MESSAGE_RATE));
    ROS_INFO("----------SeqNum: %i", msg.SeqNum);
    if (p <= static_cast<float>(rand()) / static_cast<float>(RAND_MAX))
    {
        ROS_INFO("----------AFTER SeqNum: %i", msg.SeqNum);
        //_v01_Rx_sim.header.stamp = ros::Time::now();

        // com_status.vehicle_input_rx_timeStamp = vehicle_input_rx_timeStamp;
        // com_status.vehicle_input_tx_timeStamp = vehicle_input_tx.header.stamp;
        // com_status.total_delay_sec = com_status.vehicle_input_tx_timeStamp.toNSec() - com_status.vehicle_input_rx_timeStamp.toNSec();
        // com_status.missed_msgs = 1.0*vehicle_input_tx.header.seq - 1.0*seq;
        seq = seq + 1;
        _tx_ready = true;
    }
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "itsG5_bristol");
    ros::NodeHandle n;
    n.param<double>("BAND_WIDTH", BAND_WIDTH, 6.0);      //Mb/s
    n.param<double>("MESSAGE_RATE", MESSAGE_RATE, 0.05); //seconds
    ros::Publisher v01_Rx_sim_pub = n.advertise<communication_msgs::BristolRx>("v01_Rx_sim", 1000);
    ros::Publisher status_pub = n.advertise<communication_msgs::status>("status", 1000);
    ros::Subscriber v00_Tx_sub = n.subscribe("v00_Tx_broadcast_0_215_215", 1000, input_rxCallback);

    ros::Rate loop_rate(100);
    //_v01_Rx_sim_ready = true;
    while (ros::ok())
    {
        //ros::spinOnce();

        // TODO
        // communication_msgs::status com_status;
        //ROS_INFO("----------LOOP");
        if (_tx_ready)
        {
            ROS_INFO("----------_v01_Rx_sim PUB: %i", _v01_Rx_sim.SeqNum);
            v01_Rx_sim_pub.publish(_v01_Rx_sim);
            //v01_Rx_sim_pub.publish(_v01_Rx_sim);
            _tx_ready = false;
        }
        
        //loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}