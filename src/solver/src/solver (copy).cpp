#include "ros/ros.h"
#include "cav_vehicle_model_msgs/VehicleModelInput.h"
#include "cav_vehicle_model_msgs/VehicleModelOutput.h"
#include "cav_vehicle_model_msgs/VehicleStates.h"
#include "solver_msgs/solutionHolder.h"
#include "solver_msgs/solutionHolderArr.h"
#include "solver_msgs/finalSolutionArr.h"
#include "solver_msgs/finalSolution.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include "std_msgs/String.h"

int vehicleA_seq;
int vehicleB_seq;

int SOLVER_STEP;
int predicted_final_step;

float u1_min, u1_max, w1_min, w1_max;
float u2_min, u2_max, w2_min, w2_max;
float u1, w1;
float u2, w2;
float x1_1, x1_2, x1_3, x1_4, x1_5, x1_6;
float x2_1, x2_2, x2_3, x2_4, x2_5, x2_6;
float xA_1_final, xA_2_final, xA_3_final, xA_4_final, x1_5_final, x1_6_final;
float xB_1_final, xB_2_final, xB_3_final, xB_4_final, x2_5_final, x2_6_final;
//float dx_1, dx_2, dx_3, dx_4, dx_5, dx_6;
bool vehicleA_output_msg_received = false;
bool vehicleB_output_msg_received = false;

int initial_seed_size;

solver_msgs::solutionHolderArr vehicleA_solutionsArr;
solver_msgs::solutionHolderArr vehicleB_solutionsArr;

solver_msgs::finalSolutionArr final_solution_arr;

using namespace std;

void evaluateSolutions() {
    solver_msgs::finalSolution solution;
    solution.SOLVER_STEP = SOLVER_STEP;
    solution.vehicleA_solution = vehicleA_solutionsArr.solutions[0];
    solution.vehicleB_solution = vehicleB_solutionsArr.solutions[0];

    for (int i = 0; i<=vehicleA_solutionsArr.solutions.size(); i++) {
        if (vehicleA_solutionsArr.solutions[i].fuel_cost + vehicleB_solutionsArr.solutions[i].fuel_cost
        < solution.vehicleA_solution.fuel_cost + solution.vehicleB_solution.fuel_cost
        && vehicleA_solutionsArr.solutions[i].navigation_cost + vehicleB_solutionsArr.solutions[i].navigation_cost
        < solution.vehicleA_solution.navigation_cost + solution.vehicleB_solution.navigation_cost
        && vehicleA_solutionsArr.solutions[i].collision_risk + vehicleB_solutionsArr.solutions[i].collision_risk
        < solution.vehicleA_solution.collision_risk + solution.vehicleB_solution.collision_risk) {
            solution.vehicleA_solution = vehicleA_solutionsArr.solutions[i];
            solution.vehicleB_solution = vehicleB_solutionsArr.solutions[i];
        }
    }
    final_solution_arr.solution.push_back(solution);
    // Set new initial states:
    x1_1 = solution.vehicleA_solution.vehicle_states.x_1;
    x1_2 = solution.vehicleA_solution.vehicle_states.x_2;
    x1_3 = solution.vehicleA_solution.vehicle_states.x_3;
    x1_4 = solution.vehicleA_solution.vehicle_states.x_4;
    x1_5 = solution.vehicleA_solution.vehicle_states.x_5;
    x1_6 = solution.vehicleA_solution.vehicle_states.x_6;

    x2_1 = solution.vehicleB_solution.vehicle_states.x_1;
    x2_2 = solution.vehicleB_solution.vehicle_states.x_2;
    x2_3 = solution.vehicleB_solution.vehicle_states.x_3;
    x2_4 = solution.vehicleB_solution.vehicle_states.x_4;
    x2_5 = solution.vehicleB_solution.vehicle_states.x_5;
    x2_6 = solution.vehicleB_solution.vehicle_states.x_6;
    vehicleA_seq = 0;
    vehicleB_seq = 0;
    vehicleA_output_msg_received = true;
    vehicleB_output_msg_received = true;
}

float calcNavigationCost(const cav_vehicle_model_msgs::VehicleStates s, float x1_final, float x2_final, float x3_final, float x4_final ) {
    return pow(SOLVER_STEP/(SOLVER_STEP - predicted_final_step),2)*(pow(s.x_1 - x1_final, 2) + pow(s.x_2 - x2_final, 2) + pow(s.x_3 - x3_final, 2) + pow(s.x_4 - x4_final, 2));
}

void vehicleAOutputCallback(const cav_vehicle_model_msgs::VehicleModelOutput msg)
{
    solver_msgs::solutionHolder solution;
    solution.header = msg.header;
    solution.vehicle_states = msg.vehicle_states;
    solution.vehicle_control_signals = msg.vehicle_control_signals;
    solution.fuel_cost = msg.fuel_cost;
    solution.navigation_cost = calcNavigationCost(msg.vehicle_states,xA_1_final,xA_2_final,xA_3_final,xA_4_final);
    
    vehicleA_solutionsArr.solutions.push_back(solution);
    vehicleA_output_msg_received = true;
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void vehicleBOutputCallback(const cav_vehicle_model_msgs::VehicleModelOutput msg)
{
    solver_msgs::solutionHolder solution;
    solution.header = msg.header;
    solution.vehicle_states = msg.vehicle_states;
    solution.vehicle_control_signals = msg.vehicle_control_signals;
    solution.fuel_cost = msg.fuel_cost;
    solution.navigation_cost = calcNavigationCost(msg.vehicle_states,xB_1_final,xB_2_final,xB_3_final,xB_4_final);
    
    vehicleB_solutionsArr.solutions.push_back(solution);
    vehicleB_output_msg_received = true;
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "model");
    ros::NodeHandle n;

    n.param<int>("initial_seed_size", initial_seed_size, 100);
    n.param<int>("predicted_final_step", predicted_final_step, 150);
    n.param<float>("u1_min", u1_min, 0);
    n.param<float>("u2_min", u2_min, 0);
    n.param<float>("u1_max", u1_max, 120);
    n.param<float>("u2_max", u2_max, 120);
    n.param<float>("w1_min", w1_min, -M_PI);
    n.param<float>("w2_min", w2_min, -M_PI);
    n.param<float>("w1_max", w1_max, M_PI);
    n.param<float>("w2_max", w2_max, M_PI);

    n.param<float>("x1_1_init", x1_1, 0);
    n.param<float>("x1_2_init", x1_2, 0);
    n.param<float>("x1_3_init", x1_3, 0);
    n.param<float>("x1_4_init", x1_4, 0);
    n.param<float>("x1_5_init", x1_5, 0);
    n.param<float>("x1_6_init", x1_6, 0);

    n.param<float>("x2_1_init", x2_1, 0);
    n.param<float>("x2_2_init", x2_2, 0);
    n.param<float>("x2_3_init", x2_3, 0);
    n.param<float>("x2_4_init", x2_4, 0);
    n.param<float>("x2_5_init", x2_5, 0);
    n.param<float>("x2_6_init", x2_6, 0);

    n.param<float>("xA_1_final", xA_1_final, 0);
    n.param<float>("xA_2_final", xA_2_final, 0);
    n.param<float>("xA_3_final", xA_3_final, 0);
    n.param<float>("xA_4_final", xA_4_final, 0);
    n.param<float>("x1_5_final", x1_5_final, 0);
    n.param<float>("x1_6_final", x1_6_final, 0);

    n.param<float>("xB_1_final", xB_1_final, 0);
    n.param<float>("xB_2_final", xB_2_final, 0);
    n.param<float>("xB_3_final", xB_3_final, 0);
    n.param<float>("xB_4_final", xB_4_final, 0);
    n.param<float>("x2_5_final", x2_5_final, 0);
    n.param<float>("x2_6_final", x2_6_final, 0);

    ros::Publisher vehicleA_input_pub = n.advertise<cav_vehicle_model_msgs::VehicleModelInput>("/vehicleA/input", 1000);
    ros::Publisher test_pub = n.advertise<std_msgs::String>("test", 1000);
    ros::Publisher vehicleB_input_pub = n.advertise<cav_vehicle_model_msgs::VehicleModelInput>("/vehicleB/input", 1000);
    ros::Subscriber vehicleA_output_sub = n.subscribe("vehicleA/output", 1000, vehicleAOutputCallback);
    ros::Subscriber vehicleB_output_sub = n.subscribe("vehicleB/output", 1000, vehicleBOutputCallback);

    srand (static_cast <unsigned> (time(0)));
    vehicleA_output_msg_received = true;
    vehicleB_output_msg_received = true;

    //ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::Time time_stamp = ros::Time::now();
        if (vehicleA_output_msg_received && vehicleA_seq <= initial_seed_size) {
            ROS_INFO("********************ve1");
            cav_vehicle_model_msgs::VehicleModelInput vehicle_input;
            vehicle_input.header.seq = vehicleA_seq;
            vehicle_input.header.stamp = time_stamp;
            vehicle_input.vehicle_states.x_1 = x1_1;
            vehicle_input.vehicle_states.x_2 = x1_2;
            vehicle_input.vehicle_states.x_3 = x1_3;
            vehicle_input.vehicle_states.x_4 = x1_4;
            vehicle_input.vehicle_states.x_5 = x1_5;
            vehicle_input.vehicle_states.x_6 = x1_6;
            vehicle_input.vehicle_control_signals.u = u1_min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(u1_max-u1_min)));
            vehicle_input.vehicle_control_signals.w = w1_min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(w1_max-w1_min)));
            // TODO: Need to do for orientaion as well
            std_msgs::String theString;
            theString.data = "FUCK";
            test_pub.publish(theString);
            vehicleA_input_pub.publish(vehicle_input);
            vehicleA_output_msg_received = false;
            vehicleA_seq++;
        }

        if (vehicleB_output_msg_received && vehicleB_seq <= initial_seed_size) {
            ROS_INFO("********************ve2");

            cav_vehicle_model_msgs::VehicleModelInput vehicle_input;
            vehicle_input.header.seq = vehicleB_seq;
            vehicle_input.header.stamp = time_stamp;
            vehicle_input.vehicle_states.x_1 = x2_1;
            vehicle_input.vehicle_states.x_2 = x2_2;
            vehicle_input.vehicle_states.x_3 = x2_3;
            vehicle_input.vehicle_states.x_4 = x2_4;
            vehicle_input.vehicle_states.x_5 = x2_5;
            vehicle_input.vehicle_states.x_6 = x2_6;
            vehicle_input.vehicle_control_signals.u = u2_min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(u2_max-u2_min)));
            vehicle_input.vehicle_control_signals.w = w2_min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(w2_max-w2_min)));
            // TODO: Need to do for orientaion as well
            vehicleB_input_pub.publish(vehicle_input);
            ROS_INFO("********************ve22");
            vehicleB_output_msg_received = false;
            vehicleB_seq++;
        } 
        
        if (vehicleB_seq > initial_seed_size && vehicleA_seq > initial_seed_size) {
            evaluateSolutions();
        }
        ros::spinOnce();

        //loop_rate.sleep();
        //ros::spin();
    }
    return 0;
}