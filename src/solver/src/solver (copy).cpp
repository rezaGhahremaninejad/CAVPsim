#include "ros/ros.h"
#include "vehicle_model_msgs/VehicleModelInput.h"
#include "vehicle_model_msgs/VehicleModelOutput.h"
#include "vehicle_model_msgs/VehicleStates.h"
#include "solver_msgs/solutionHolder.h"
#include "solver_msgs/solutionHolderArr.h"
#include "solver_msgs/finalSolutionArr.h"
#include "solver_msgs/finalSolution.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include "std_msgs/String.h"

int vehicle1_seq;
int vehicle2_seq;

int solution_step;
int predicted_final_step;

float u1_min, u1_max, w1_min, w1_max;
float u2_min, u2_max, w2_min, w2_max;
float u1, w1;
float u2, w2;
float x1_1, x1_2, x1_3, x1_4, x1_5, x1_6;
float x2_1, x2_2, x2_3, x2_4, x2_5, x2_6;
float x1_1_final, x1_2_final, x1_3_final, x1_4_final, x1_5_final, x1_6_final;
float x2_1_final, x2_2_final, x2_3_final, x2_4_final, x2_5_final, x2_6_final;
//float dx_1, dx_2, dx_3, dx_4, dx_5, dx_6;
bool vehicle1_output_msg_received = false;
bool vehicle2_output_msg_received = false;

int initial_seed_size;

solver_msgs::solutionHolderArr vehicle1_solutionsArr;
solver_msgs::solutionHolderArr vehicle2_solutionsArr;

solver_msgs::finalSolutionArr final_solution_arr;

using namespace std;

void evaluateSolutions() {
    solver_msgs::finalSolution solution;
    solution.solution_step = solution_step;
    solution.vehicle1_solution = vehicle1_solutionsArr.solutions[0];
    solution.vehicle2_solution = vehicle2_solutionsArr.solutions[0];

    for (int i = 0; i<=vehicle1_solutionsArr.solutions.size(); i++) {
        if (vehicle1_solutionsArr.solutions[i].fuel_cost + vehicle2_solutionsArr.solutions[i].fuel_cost
        < solution.vehicle1_solution.fuel_cost + solution.vehicle2_solution.fuel_cost
        && vehicle1_solutionsArr.solutions[i].navigation_cost + vehicle2_solutionsArr.solutions[i].navigation_cost
        < solution.vehicle1_solution.navigation_cost + solution.vehicle2_solution.navigation_cost
        && vehicle1_solutionsArr.solutions[i].collision_risk + vehicle2_solutionsArr.solutions[i].collision_risk
        < solution.vehicle1_solution.collision_risk + solution.vehicle2_solution.collision_risk) {
            solution.vehicle1_solution = vehicle1_solutionsArr.solutions[i];
            solution.vehicle2_solution = vehicle2_solutionsArr.solutions[i];
        }
    }
    final_solution_arr.solution.push_back(solution);
    // Set new initial states:
    x1_1 = solution.vehicle1_solution.vehicle_states.x_1;
    x1_2 = solution.vehicle1_solution.vehicle_states.x_2;
    x1_3 = solution.vehicle1_solution.vehicle_states.x_3;
    x1_4 = solution.vehicle1_solution.vehicle_states.x_4;
    x1_5 = solution.vehicle1_solution.vehicle_states.x_5;
    x1_6 = solution.vehicle1_solution.vehicle_states.x_6;

    x2_1 = solution.vehicle2_solution.vehicle_states.x_1;
    x2_2 = solution.vehicle2_solution.vehicle_states.x_2;
    x2_3 = solution.vehicle2_solution.vehicle_states.x_3;
    x2_4 = solution.vehicle2_solution.vehicle_states.x_4;
    x2_5 = solution.vehicle2_solution.vehicle_states.x_5;
    x2_6 = solution.vehicle2_solution.vehicle_states.x_6;
    vehicle1_seq = 0;
    vehicle2_seq = 0;
    vehicle1_output_msg_received = true;
    vehicle2_output_msg_received = true;
}

float calcNavigationCost(const vehicle_model_msgs::VehicleStates s, float x1_final, float x2_final, float x3_final, float x4_final ) {
    return pow(solution_step/(solution_step - predicted_final_step),2)*(pow(s.x_1 - x1_final, 2) + pow(s.x_2 - x2_final, 2) + pow(s.x_3 - x3_final, 2) + pow(s.x_4 - x4_final, 2));
}

void vehicle1OutputCallback(const vehicle_model_msgs::VehicleModelOutput msg)
{
    solver_msgs::solutionHolder solution;
    solution.header = msg.header;
    solution.vehicle_states = msg.vehicle_states;
    solution.vehicle_control_signals = msg.vehicle_control_signals;
    solution.fuel_cost = msg.fuel_cost;
    solution.navigation_cost = calcNavigationCost(msg.vehicle_states,x1_1_final,x1_2_final,x1_3_final,x1_4_final);
    
    vehicle1_solutionsArr.solutions.push_back(solution);
    vehicle1_output_msg_received = true;
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void vehicle2OutputCallback(const vehicle_model_msgs::VehicleModelOutput msg)
{
    solver_msgs::solutionHolder solution;
    solution.header = msg.header;
    solution.vehicle_states = msg.vehicle_states;
    solution.vehicle_control_signals = msg.vehicle_control_signals;
    solution.fuel_cost = msg.fuel_cost;
    solution.navigation_cost = calcNavigationCost(msg.vehicle_states,x2_1_final,x2_2_final,x2_3_final,x2_4_final);
    
    vehicle2_solutionsArr.solutions.push_back(solution);
    vehicle2_output_msg_received = true;
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

    n.param<float>("x1_1_final", x1_1_final, 0);
    n.param<float>("x1_2_final", x1_2_final, 0);
    n.param<float>("x1_3_final", x1_3_final, 0);
    n.param<float>("x1_4_final", x1_4_final, 0);
    n.param<float>("x1_5_final", x1_5_final, 0);
    n.param<float>("x1_6_final", x1_6_final, 0);

    n.param<float>("x2_1_final", x2_1_final, 0);
    n.param<float>("x2_2_final", x2_2_final, 0);
    n.param<float>("x2_3_final", x2_3_final, 0);
    n.param<float>("x2_4_final", x2_4_final, 0);
    n.param<float>("x2_5_final", x2_5_final, 0);
    n.param<float>("x2_6_final", x2_6_final, 0);

    ros::Publisher vehicle1_input_pub = n.advertise<vehicle_model_msgs::VehicleModelInput>("/vehicle1/input", 1000);
    ros::Publisher test_pub = n.advertise<std_msgs::String>("test", 1000);
    ros::Publisher vehicle2_input_pub = n.advertise<vehicle_model_msgs::VehicleModelInput>("/vehicle2/input", 1000);
    ros::Subscriber vehicle1_output_sub = n.subscribe("vehicle1/output", 1000, vehicle1OutputCallback);
    ros::Subscriber vehicle2_output_sub = n.subscribe("vehicle2/output", 1000, vehicle2OutputCallback);

    srand (static_cast <unsigned> (time(0)));
    vehicle1_output_msg_received = true;
    vehicle2_output_msg_received = true;

    //ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::Time time_stamp = ros::Time::now();
        if (vehicle1_output_msg_received && vehicle1_seq <= initial_seed_size) {
            ROS_INFO("********************ve1");
            vehicle_model_msgs::VehicleModelInput vehicle_input;
            vehicle_input.header.seq = vehicle1_seq;
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
            vehicle1_input_pub.publish(vehicle_input);
            vehicle1_output_msg_received = false;
            vehicle1_seq++;
        }

        if (vehicle2_output_msg_received && vehicle2_seq <= initial_seed_size) {
            ROS_INFO("********************ve2");

            vehicle_model_msgs::VehicleModelInput vehicle_input;
            vehicle_input.header.seq = vehicle2_seq;
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
            vehicle2_input_pub.publish(vehicle_input);
            ROS_INFO("********************ve22");
            vehicle2_output_msg_received = false;
            vehicle2_seq++;
        } 
        
        if (vehicle2_seq > initial_seed_size && vehicle1_seq > initial_seed_size) {
            evaluateSolutions();
        }
        ros::spinOnce();

        //loop_rate.sleep();
        //ros::spin();
    }
    return 0;
}