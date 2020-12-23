#include "ros/ros.h"
#include "computation_msgs/status.h"

#include <ctime>
#include <cmath>

double batch_size, t_op, app_flo;

int seq;

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "compute_model");
    ros::NodeHandle n;
    n.param<double>("batch_size", batch_size, 10000);
    n.param<double>("t_op", t_op, 0.01);
    n.param<double>("app_flo", app_flo, 10000);

    ros::Publisher status_pub = n.advertise<computation_msgs::status>("status", 1000);

    ros::Rate loop_rate(1000);

    while (ros::ok())
    {
        ros::spinOnce();
        // TODO
        ros::Time t0, t1;
        t0 = ros::Time::now();
        //ROS_INFO("########################START");
        for (int i = 0; i <= app_flo*batch_size; i++){
            float res = 3.14*3.13; //1 floating point operation
        }
        //ROS_INFO("########################Finish");
        t1 = ros::Time::now();
        computation_msgs::status comp_status;
        comp_status.header.seq = seq;
        comp_status.header.stamp = t1;
        comp_status.batch_size = batch_size;
        comp_status.t_op = t_op;
        comp_status.app_flo = app_flo;
        comp_status.execution_time = (t1-t0).toSec();
        comp_status.required_flops = app_flo*batch_size*t_op;
        comp_status.machine_availabel_flops = app_flo*batch_size/(t1-t0).toSec();
        //ros::Duration d(delay_sec);
        status_pub.publish(comp_status);
        seq = seq + 1;
        //if (ros::Time::now() >= vehicle_output_rx_timeStamp + d && vehicle_output_tx_ready)

        //loop_rate.sleep();
    }
    return 0;
}