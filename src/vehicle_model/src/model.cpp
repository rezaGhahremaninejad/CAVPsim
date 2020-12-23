#include "ros/ros.h"
#include "model_msgs/VehicleModelInput.h"
#include "model_msgs/VehicleModelOutput.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <cmath>

float L, m, ga, dt, R, I_d, I_g, Im_e, Im_t, Im_w, GR, ideal_fuel_cost;
float u, w;
float x_1, x_2, x_3, x_4, x_5, x_6, x_7;
float B_1, B_2, B_3, B_4, B_5, B_6;
float rolling_res_force;

//float dx_1, dx_2, dx_3, dx_4, dx_5, dx_6;
bool solver_req = false;
bool msg_received = false;
int seq;
float solver_time_step;

using namespace std;

void inputCallback(const model_msgs::VehicleModelInput::ConstPtr &msg)
{
    seq = msg->header.seq;
    if (msg->useThisStates)
    {
        x_1 = msg->vehicle_states.x_1;
        x_2 = msg->vehicle_states.x_2;
        x_3 = msg->vehicle_states.x_3;
        x_4 = msg->vehicle_states.x_4;
        x_5 = msg->vehicle_states.x_5;
        x_6 = msg->vehicle_states.x_6;
        x_7 = msg->vehicle_states.x_7;
        solver_req = true;
        solver_time_step = msg->solver_time_step;
    }

    u = msg->vehicle_control_signals.u;
    w = msg->vehicle_control_signals.w;
    msg_received = true;

    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "model");
    ros::NodeHandle n;
    n.param<float>("L", L, 2.7);                             //wheelbase
    n.param<float>("m", m, 1200);                            // mass
    n.param<float>("dt", dt, 0.01);                           // sampling time
    n.param<float>("R", R, 0.19);                            //  effective redius
    n.param<float>("I_d", I_d, 0.3);                         // differential ratio
    n.param<float>("I_g", I_g, 0.3);                         // gear ratio
    n.param<float>("Im_e", Im_e, 0.3);                       // engine rotation momentume
    n.param<float>("Im_t", Im_t, 0.3);                       // turbine rotation momentume
    n.param<float>("Im_w", Im_w, 0.3);                       // wheel rotation momentume
    n.param<float>("solver_time_step", solver_time_step, 0.1); // wheel rotation momentume
    n.param<bool>("solver_req", solver_req, true);           // wheel rotation momentume
    n.param<float>("ideal_fuel_cost", ideal_fuel_cost, 0.01);           // wheel rotation momentume

    //n.param<float>("GR", GR, 0.3);
    GR = I_d * I_g;
    B_1 = 1.1046 * 0.01 * I_d / R;
    B_2 = -7.7511 * (pow(10, -5)) * (pow(I_d, 2)) / pow(R, 2);
    B_3 = 1.6958 * (pow(10, -7)) * (pow(I_d, 3)) / pow(R, 3);
    B_4 = 1.7363 * (pow(10, -5)) / R;
    B_5 = 6.4277 * (pow(10, -8)) * I_d / pow(R, 2);
    B_6 = 1.6088 * (pow(10, -7)) / (R * I_d);
    ros::Publisher ouput_pub = n.advertise<model_msgs::VehicleModelOutput>("output", 1000);
    ros::Publisher nav_pub = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    ros::Subscriber input_sub = n.subscribe("input", 1000, inputCallback);

    tf::TransformBroadcaster odom_broadcaster;

    ros::Rate loop_rate(1 / dt);

    if (solver_req)
    {
        dt = solver_time_step;
        ROS_INFO("########################y:");
    }

    while (ros::ok())
    {
        ros::spinOnce();

        if (msg_received)
        {
            nav_msgs::Odometry odometry_msg;
            odometry_msg.header.seq = seq;
            odometry_msg.header.stamp = ros::Time::now();
            odometry_msg.header.frame_id = "odom";
            odometry_msg.child_frame_id = "base_link";

            model_msgs::VehicleModelOutput output;
            output.header.seq = seq;
            ROS_INFO("######################## Received message sequence: [%d]", seq);
            output.header.stamp = odometry_msg.header.stamp;

            // REF: https://www.coursera.org/lecture/intro-self-driving-cars/lesson-4-longitudinal-vehicle-modeling-V8htX
            float R_x = 0.1 * m * abs(x_4);   // Rolling resistan force N
            float F_aero = 0.1 * pow(x_4, 2); // Rolling resistan force N
            float F_load = F_aero + R_x;
            float je = Im_e + Im_t + Im_w * pow(GR, 2) + m * pow(GR, 2) * pow(R, 2);
            float dwheel_rot = (u - GR * R * F_load) / je;
            output.vehicle_states.x_5 = R * GR * dwheel_rot; //acceleration m/s^2
            output.vehicle_states.x_7 = x_7 + dt * w;        //front wheel angle
            output.vehicle_states.x_6 = atan(tan(x_7) / L);  // rad
            output.vehicle_states.x_4 = x_4 + dt * output.vehicle_states.x_5;
            output.vehicle_states.x_3 = x_3 + dt * (1.0 / L) * output.vehicle_states.x_4 * sin(output.vehicle_states.x_6); //rad
            output.vehicle_states.x_1 = x_1 + dt * output.vehicle_states.x_4 * cos(output.vehicle_states.x_3);
            output.vehicle_states.x_2 = x_2 + dt * output.vehicle_states.x_4 * sin(output.vehicle_states.x_3);

            x_1 = output.vehicle_states.x_1; //pos in x
            x_2 = output.vehicle_states.x_2; //pos in y
            x_3 = output.vehicle_states.x_3; //inertial heading rad
            x_4 = output.vehicle_states.x_4; //speed m/s
            x_5 = output.vehicle_states.x_5; // acceleration m/s^2
            x_6 = output.vehicle_states.x_6; // slip angle, the angle of the current velocity of the center of mass with respec to thr longitudinal axis of the car
            x_7 = output.vehicle_states.x_7; //steer wheel angle rad

            output.vehicle_control_signals.u = u;
            output.vehicle_control_signals.w = w;
            geometry_msgs::Pose pose;
            pose.position.x = x_1;
            pose.position.y = x_2;
            //ROS_INFO("########################y: [%f]", output.vehicle_states.x_3);
            geometry_msgs::Quaternion myQuaternion = tf::createQuaternionMsgFromYaw(x_3);
            pose.orientation = myQuaternion;

            geometry_msgs::Twist twist;
            twist.linear.x = x_4 * cos(x_3);
            twist.linear.y = x_4 * sin(x_3);
            twist.angular.z = (1.0 / L) * x_4 * sin(x_6);

            odometry_msg.pose.pose = pose;
            odometry_msg.twist.twist = twist;
            output.pose = pose;
            // TODO: Need to do for orientaion as well
            if (u <= 0) {
                output.fuel_cost = ideal_fuel_cost;
            } else {
                output.fuel_cost = I_g * x_4 * (B_1 + B_2 * I_g * x_4 + B_3 * pow(I_g, 2) * pow(x_4, 2)) + B_4 * u * x_4 + B_5 * pow(x_4, 2) * u * I_g + B_6 * x_4 * pow(u, 2) / I_g ;
            }
            
            ouput_pub.publish(output);

            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = odometry_msg.header.stamp;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = x_1;
            odom_trans.transform.translation.y = x_2;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = myQuaternion;

            //send the transform
            odom_broadcaster.sendTransform(odom_trans);

            nav_pub.publish(odometry_msg);
            msg_received = false;
        }

        loop_rate.sleep();
    }
    return 0;
}