#include "ros/ros.h"
#include "cav_vehicle_model_msgs/VehicleModelInput.h"
#include "cav_vehicle_model_msgs/VehicleModelOutput.h"
#include <cmath>

float x_1, x_2, x_3, x_4, x_5, x_6;
float T_1, T_2, T_3, T_4, T_5, T_6, T_7, T_1_U, T_1_W, T_2_U, T_2_W, 
T_3_U, T_3_W, T_4_U, T_4_W, T_5_U, T_5_W, T_6_U, T_6_W, INITIAL_U, INITIAL_W, FINAL_U, FINAL_W;
//float dx_1, dx_2, dx_3, dx_4, dx_5, dx_6;
bool feedback_msg_received = false;

int the_seq;

float dt;

using namespace std;

void feedBackCallback(const cav_vehicle_model_msgs::VehicleModelOutput::ConstPtr &msg)
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
    
    ros::Publisher output_pub = n.advertise<cav_vehicle_model_msgs::VehicleModelInput>("input", 1000);
    // ros::Subscriber vehicle_feedback_sub = n.subscribe("ouput", 1000, feedBackCallback);
    n.param<float>("dummy_control/T_1", T_1, 0.01);           
    n.param<float>("dummy_control/T_2", T_2, 0.01);           
    n.param<float>("dummy_control/T_3", T_3, 0.01);           
    n.param<float>("dummy_control/T_4", T_4, 0.01);           
    n.param<float>("dummy_control/T_5", T_5, 0.01);           
    n.param<float>("dummy_control/T_6", T_6, 0.01);           
    n.param<float>("dummy_control/T_7", T_6, 0.01);           
    n.param<float>("dummy_control/T_1_U", T_1_U, 0.01);           
    n.param<float>("dummy_control/T_1_W", T_1_W, 0.01);           
    n.param<float>("dummy_control/T_2_U", T_2_U, 0.01);           
    n.param<float>("dummy_control/T_2_W", T_2_W, 0.01); 
    n.param<float>("dummy_control/T_3_U", T_3_U, 0.01);           
    n.param<float>("dummy_control/T_3_W", T_3_W, 0.01);
    n.param<float>("dummy_control/T_4_U", T_4_U, 0.01);           
    n.param<float>("dummy_control/T_4_W", T_4_W, 0.01);
    n.param<float>("dummy_control/T_5_U", T_5_U, 0.01);           
    n.param<float>("dummy_control/T_5_W", T_5_W, 0.01);
    n.param<float>("dummy_control/T_6_U", T_5_U, 0.01);           
    n.param<float>("dummy_control/T_6_W", T_5_W, 0.01);
    n.param<float>("dummy_control/INITIAL_U", INITIAL_U, 0.01);           
    n.param<float>("dummy_control/INITIAL_W", INITIAL_W, 0.01);           
    n.param<float>("dummy_control/FINAL_U", FINAL_U, 0.01);           
    n.param<float>("dummy_control/FINAL_W", FINAL_W, 0.01);           

    ros::Rate loop_rate(1000);
    ros::Time start_time = ros::Time::now();

    while (ros::ok())
    {
        ros::Time time_stamp = ros::Time::now();
        cav_vehicle_model_msgs::VehicleModelInput vehicle_input;
        vehicle_input.header.seq = the_seq;
        vehicle_input.header.stamp = time_stamp;
        vehicle_input.useThisStates = false;

        ros::Duration dur = time_stamp - start_time;
        // Minimum torque corresponding to zero engine torque should be -10 or something to guarantee friction effect


        if (dur.toSec() < T_1) {
            vehicle_input.vehicle_control_signals.u = INITIAL_U;
            vehicle_input.vehicle_control_signals.w = INITIAL_W;
            // std::cout << "-------------TIME DIFF 1: " << dur.toSec() << std::endl;

        } else if (dur.toSec() < T_2) {
            // std::cout << "-------------TIME DIFF 2: " << dur.toSec() << std::endl;
            vehicle_input.vehicle_control_signals.u = T_1_U;
            vehicle_input.vehicle_control_signals.w = T_1_W;
        } else if (dur.toSec() < T_3) {
            // std::cout << "-------------TIME DIFF 3: " << dur.toSec() << std::endl;
            vehicle_input.vehicle_control_signals.u = T_2_U;
            vehicle_input.vehicle_control_signals.w = T_2_W;
        } else if (dur.toSec() < T_4) {
            // std::cout << "-------------TIME DIFF 4: " << dur.toSec() << std::endl;
            vehicle_input.vehicle_control_signals.u = T_3_U;
            vehicle_input.vehicle_control_signals.w = T_3_W;
        } else if (dur.toSec() < T_5) {
            // std::cout << "-------------TIME DIFF 5: " << dur.toSec() << std::endl;
            vehicle_input.vehicle_control_signals.u = T_4_U;
            vehicle_input.vehicle_control_signals.w = T_4_W;
        } else if (dur.toSec() < T_6) {
            // std::cout << "-------------TIME DIFF 6: " << dur.toSec() << std::endl;
            vehicle_input.vehicle_control_signals.u = T_5_U;
            vehicle_input.vehicle_control_signals.w = T_5_W;
        } else if (dur.toSec() < T_7) {
            // std::cout << "-------------TIME DIFF 7: " << dur.toSec() << std::endl;
            vehicle_input.vehicle_control_signals.u = T_6_U;
            vehicle_input.vehicle_control_signals.w = T_6_W;
        } else {
            // std::cout << "-------------TIME DIFF 8: " << dur.toSec() << std::endl;
            // std::cout << "-------------T_1: " << T_1 << std::endl;
            vehicle_input.vehicle_control_signals.u = FINAL_U;
            vehicle_input.vehicle_control_signals.w = FINAL_W;
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