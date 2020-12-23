#include "ros/ros.h"
#include "model_msgs/VehicleModelInput.h"
#include "model_msgs/VehicleModelOutput.h"
#include <cmath>

float x_1, x_2, x_3, x_4, x_5, x_6;
//float dx_1, dx_2, dx_3, dx_4, dx_5, dx_6;
bool feedback_msg_received = false;

int the_seq;

float dt;

using namespace std;

void feedBackCallback(const model_msgs::VehicleModelOutput::ConstPtr &msg)
{
    the_seq = msg->header.seq;
    x_1 = msg->vehicle_states.x_1;
    x_2 = msg->vehicle_states.x_2;
    x_3 = msg->vehicle_states.x_3;
    x_4 = msg->vehicle_states.x_4;
    x_5 = msg->vehicle_states.x_5;
    x_6 = msg->vehicle_states.x_6;

    //u = msg->vehicle_control_signals.u;
    //w = msg->vehicle_control_signals.w;
    feedback_msg_received = true;
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_control");
    ros::NodeHandle n;
    n.param<float>("dt", dt, 0.00000025);
    
    ros::Publisher output_pub = n.advertise<model_msgs::VehicleModelInput>("input", 1000);
    ros::Subscriber vehicle_feedback_sub = n.subscribe("ouput", 1000, feedBackCallback);

    ros::Rate loop_rate(1000);
    ros::Time start_time = ros::Time::now();

    while (ros::ok())
    {
        ros::Time time_stamp = ros::Time::now();
        model_msgs::VehicleModelInput vehicle_input;
        vehicle_input.header.seq = the_seq;
        vehicle_input.header.stamp = time_stamp;
        vehicle_input.useThisStates = false;

        ros::Duration dur = time_stamp - start_time;
        // Minimum torque corresponding to zero engine torque should be -10 or something to guarantee friction effect


        if (dur.toSec() < 10) {
            vehicle_input.vehicle_control_signals.u = 80;
            vehicle_input.vehicle_control_signals.w = 0;

        } else if (dur.toSec() > 10 && dur.toSec() < 12) {
            vehicle_input.vehicle_control_signals.u = 0;
            vehicle_input.vehicle_control_signals.w = 0.02;
        } else if (dur.toSec() > 12  && dur.toSec() < 13) {
            vehicle_input.vehicle_control_signals.u = 0;
            vehicle_input.vehicle_control_signals.w = -0.02;
        } else if (dur.toSec() > 13  && dur.toSec() < 20) {
            vehicle_input.vehicle_control_signals.u = 30;
            vehicle_input.vehicle_control_signals.w = 0;
        } else if (dur.toSec() > 20 && dur.toSec() < 22) {
            vehicle_input.vehicle_control_signals.u = 0;
            vehicle_input.vehicle_control_signals.w = 0;
        } else if (dur.toSec() > 22 && dur.toSec() < 26) {
            vehicle_input.vehicle_control_signals.u = -30;
            vehicle_input.vehicle_control_signals.w = -0.02;
        } else {
            vehicle_input.vehicle_control_signals.u = 0;
            vehicle_input.vehicle_control_signals.w = 0;
        }
            
        //vehicle_input.vehicle_control_signals.u = u_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (u1_max - u1_min)));
        //vehicle_input.vehicle_control_signals.w = w_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (w1_max - w1_min)));
        //  TODO: Need to do for orientaion as well
        output_pub.publish(vehicle_input);
        ros::spinOnce();
        //ros::spin();
        loop_rate.sleep();
        the_seq = the_seq + 1;
    }
    return 0;
}