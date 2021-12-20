#include "ros/ros.h"
#include "cav_vehicle_model_msgs/VehicleModelInput.h"
#include "cav_vehicle_model_msgs/VehicleModelOutput.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "vehicleModel.c"
#include "vehicleModel_data.c"
#include "rt_nonfinite.c"
#include "rtGetNaN.c"
#include "rtGetInf.c"
#include "rt_defines.h"
#include "rtmodel.h"
// #include "rt_logging.c"
#include "multiword_types.h"
// #include "rt_malloc_main.c"

float L, m, ga, dt, R, I_d, I_g, Im_e, Im_t, Im_w, GR, ideal_fuel_cost;
float u, w;
float x_1_init, x_2_init, x_3_init;
float x_1, x_2, x_3, x_4, x_5, x_6, x_7;
float B_1, B_2, B_3, B_4, B_5, B_6;
float rolling_res_force;

//float dx_1, dx_2, dx_3, dx_4, dx_5, dx_6;
bool solver_req = false;
bool msg_received = false;
int seq;
float solver_time_step;
std::string base_link;
using namespace std;
const char *RT_MEMORY_ALLOCATION_ERROR = "memory allocation error"; 

void inputCallback(const cav_vehicle_model_msgs::VehicleModelInput::ConstPtr &msg)
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
    n.param<float>("model/L", L, 2.7);                                  // wheelbase
    n.param<float>("model/m", m, 1200);                                 // mass
    n.param<float>("model/dt", dt, 0.01);                               // sampling time
    n.param<float>("model/R", R, 0.19);                                 // effective redius
    n.param<float>("model/I_d", I_d, 0.3);                              // differential ratio
    n.param<float>("model/I_g", I_g, 0.3);                              // gear ratio
    n.param<float>("model/Im_e", Im_e, 0.3);                            // engine rotation momentume
    n.param<float>("model/Im_t", Im_t, 0.3);                            // turbine rotation momentume
    n.param<float>("model/Im_w", Im_w, 0.3);                            // wheel rotation momentume
    n.param<float>("model/solver_time_step", solver_time_step, 0.1);    // time step
    n.param<bool>("model/solver_req", solver_req, true);                // if solver requested 
    n.param<float>("model/ideal_fuel_cost", ideal_fuel_cost, 0.01);     
    n.param<float>("model/initial_x", x_1_init, 0.01);                  
    n.param<float>("model/initial_y", x_2_init, 0.01);                  
    n.param<float>("model/initial_heading", x_3_init, 0.01);            
    n.param<float>("model/initial_speed", x_4, 0.01);                   
    n.param<std::string>("model/base_link", base_link, "base_link");    

    ros::Publisher ouput_pub = n.advertise<cav_vehicle_model_msgs::VehicleModelOutput>("/cav_vehicle_model/output", 1000);
    ros::Publisher nav_pub = n.advertise<nav_msgs::Odometry>("/cav_vehicle_model/odometry", 1000);
    ros::Subscriber input_sub = n.subscribe("/cav_vehicle_model/input", 1000, inputCallback);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "/cav_vehicle_model/marker", 0 );     
    ros::Publisher current_pose_pub = n.advertise<geometry_msgs::PoseStamped>( "/cav_vehicle_model/current_pose", 0 );     
    ros::Publisher current_velocity_pub = n.advertise<geometry_msgs::TwistStamped>( "/cav_vehicle_model/current_velocity", 0 );     
    tf::TransformBroadcaster odom_broadcaster;

    ros::Rate loop_rate(1 / dt);

    if (solver_req)
    {
        dt = solver_time_step;
        ROS_INFO("########################y:");
    }
    
    
    msg_received = true;
    auto _model = vehicleModel();

    //initializing the vehicleModel:
    vehicleModel_initialize(_model);
    
    while (ros::ok())
    {
        ros::spinOnce();
        _model->inputs->FwR = u;
        _model->inputs->FwF = u;
        _model->inputs->steeringRate = w;
        vehicleModel_step(_model);
        // std::cout << "---------------------------------------------" << std::endl;
        // std::cout << "---_model->outputs->state[0]: " << _model->outputs->state[0] << std::endl;
        // std::cout << "---_model->outputs->state[1]: " << _model->outputs->state[1] << std::endl;
        // std::cout << "---_model->outputs->state[2]: " << _model->outputs->state[2] << std::endl;
        // std::cout << "---_model->outputs->stateDot[0]: " << _model->outputs->stateDot[0] << std::endl;
        // std::cout << "---_model->outputs->stateDot[1]: " << _model->outputs->stateDot[1] << std::endl;
        // std::cout << "---_model->outputs->stateDot[2]: " << _model->outputs->stateDot[2] << std::endl;
        
        if (msg_received)
        {
            nav_msgs::Odometry odometry_msg;
            odometry_msg.header.seq = seq;
            odometry_msg.header.stamp = ros::Time::now();
            odometry_msg.header.frame_id = "odom";
            odometry_msg.child_frame_id = base_link;

            cav_vehicle_model_msgs::VehicleModelOutput output;
            output.header.seq = seq;
            // ROS_INFO("######################## Received message sequence: [%d]", seq);
            output.header.stamp = odometry_msg.header.stamp;

            x_1 = _model->outputs->state[0] + x_1_init;                        //pos in x
            x_2 = _model->outputs->state[1] + x_2_init;                        //pos in y 
            x_3 = _model->outputs->state[2] + x_3_init;                        //inertial heading rad
            x_4 = pow(pow(_model->outputs->stateDot[0],2) 
            + pow(_model->outputs->stateDot[1],2), 0.5);            //speed m/s
            // x_5 = output.vehicle_states.x_5; // acceleration m/s^2
            // x_6 = output.vehicle_states.x_6; // slip angle, the angle of the current velocity of the center of mass with respec to thr longitudinal axis of the car
            x_7 += w*dt;                                            //steer wheel angle rad
            
            // x_1 = output.vehicle_states.x_1; //pos in x
            // x_2 = output.vehicle_states.x_2; //pos in y
            // x_3 = output.vehicle_states.x_3; //inertial heading rad
            // x_4 = output.vehicle_states.x_4; //speed m/s
            // x_5 = output.vehicle_states.x_5; // acceleration m/s^2
            // x_6 = output.vehicle_states.x_6; // slip angle, the angle of the current velocity of the center of mass with respec to thr longitudinal axis of the car
            // x_7 = output.vehicle_states.x_7; //steer wheel angle rad

            output.vehicle_control_signals.u = u;
            output.vehicle_control_signals.w = w;
            geometry_msgs::Pose pose;
            pose.position.x = x_1;
            pose.position.y = x_2;
            //ROS_INFO("########################y: [%f]", output.vehicle_states.x_3);
            geometry_msgs::Quaternion myQuaternion = tf::createQuaternionMsgFromYaw(x_3);
            pose.orientation = myQuaternion;
            geometry_msgs::PoseStamped _pose_stamped;
            _pose_stamped.header.stamp = ros::Time::now();
            _pose_stamped.pose = pose;
            current_pose_pub.publish(_pose_stamped);

            geometry_msgs::Twist twist;
            twist.linear.x = _model->outputs->stateDot[0];
            twist.linear.y = _model->outputs->stateDot[1];
            // twist.linear.x = x_4 * cos(x_3);
            // twist.linear.y = x_4 * sin(x_3);
            // twist.angular.z = (1.0 / L) * x_4 * sin(x_6);
            twist.angular.z = (1.0 / L) * x_4 * sin(x_3);
            geometry_msgs::TwistStamped _twist_stamped;
            _twist_stamped.header.stamp = ros::Time::now();
            _twist_stamped.twist = twist;
            current_velocity_pub.publish(_twist_stamped);

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
            odom_trans.child_frame_id = base_link;

            odom_trans.transform.translation.x = x_1;
            odom_trans.transform.translation.y = x_2;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = myQuaternion;

            //send the transform
            odom_broadcaster.sendTransform(odom_trans);

            nav_pub.publish(odometry_msg);
            visualization_msgs::Marker marker;
            marker.header.frame_id = base_link;
            marker.header.stamp = ros::Time();
            marker.ns = base_link + "_ns";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            // marker.pose = pose;
            // marker.pose.orientation = myQuaternion;
            marker.scale.x = 6;
            marker.scale.y = 2.4;
            marker.scale.z = 2.2;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            //only if using a MESH_RESOURCE marker type:
            // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
            vis_pub.publish( marker );
            msg_received = false;
        }

        loop_rate.sleep();
    }
    return 0;
}