#include "ros/ros.h"
#include "model_msgs/VehicleModelInput.h"
#include "model_msgs/VehicleModelOutput.h"
#include "model_msgs/VehicleStates.h"
#include "solver_msgs/solutionHolder.h"
#include "solver_msgs/solutionHolderArr.h"
#include "solver_msgs/finalSolutionArr.h"
#include "solver_msgs/finalSolution.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include "std_msgs/String.h"

bool preccessEnd = false;

int the_seq = 1;

int solution_step; 
int vehicel1_output_seq;
int vehicel2_output_seq;
int predicted_final_step;
float threshold;
float solver_time_step;
int max_step;

float u1_min, u1_max, w1_min, w1_max;
float u2_min, u2_max, w2_min, w2_max;
float u1, w1;
float u2, w2;
float x1_1, x1_2, x1_3, x1_4, x1_5, x1_6, x1_7;
float x2_1, x2_2, x2_3, x2_4, x2_5, x2_6, x2_7;
float x1_1_final, x1_2_final, x1_3_final, x1_4_final, x1_5_final, x1_6_final, x1_7_final;
float x2_1_final, x2_2_final, x2_3_final, x2_4_final, x2_5_final, x2_6_final, x2_7_final;
//float dx_1, dx_2, dx_3, dx_4, dx_5, dx_6;
bool vehicle1_output_msg_received = false;
bool vehicle2_output_msg_received = false;

int initial_seed_size;
solver_msgs::solutionHolderArr vehicle1_solutionsArr;
solver_msgs::solutionHolderArr vehicle2_solutionsArr;

solver_msgs::finalSolutionArr final_solution_arr;

using namespace std;

void evaluateSolutions()
{
    solver_msgs::finalSolution solution;
    solution.solution_step = solution_step;
    //ROS_INFO("----------------- size: [%f]", vehicle1_solutionsArr.solutions[initial_seed_size/2].vehicle_control_signals.u);
    solution.vehicle1_solution = vehicle1_solutionsArr.solutions[initial_seed_size/2];
    solution.vehicle2_solution = vehicle2_solutionsArr.solutions[initial_seed_size/2];

    for (int i = 0; i < vehicle1_solutionsArr.solutions.size(); i++)
    {
        //if (vehicle1_solutionsArr.solutions[i].fuel_cost + vehicle2_solutionsArr.solutions[i].fuel_cost < solution.vehicle1_solution.fuel_cost + solution.vehicle2_solution.fuel_cost && vehicle1_solutionsArr.solutions[i].navigation_cost + vehicle2_solutionsArr.solutions[i].navigation_cost < solution.vehicle1_solution.navigation_cost + solution.vehicle2_solution.navigation_cost && vehicle1_solutionsArr.solutions[i].collision_risk + vehicle2_solutionsArr.solutions[i].collision_risk < solution.vehicle1_solution.collision_risk + solution.vehicle2_solution.collision_risk)
        //if (vehicle1_solutionsArr.solutions[i].navigation_cost + vehicle2_solutionsArr.solutions[i].navigation_cost < solution.vehicle1_solution.navigation_cost + solution.vehicle2_solution.navigation_cost)
        if (vehicle1_solutionsArr.solutions[i].fuel_cost + vehicle2_solutionsArr.solutions[i].fuel_cost < solution.vehicle1_solution.fuel_cost + solution.vehicle2_solution.fuel_cost && vehicle1_solutionsArr.solutions[i].navigation_cost + vehicle2_solutionsArr.solutions[i].navigation_cost < solution.vehicle1_solution.navigation_cost + solution.vehicle2_solution.navigation_cost)
        //if (vehicle1_solutionsArr.solutions[i].navigation_cost + vehicle2_solutionsArr.solutions[i].navigation_cost < solution.vehicle1_solution.navigation_cost + solution.vehicle2_solution.navigation_cost)
        {
            solution.vehicle1_solution = vehicle1_solutionsArr.solutions[i];
            solution.vehicle2_solution = vehicle2_solutionsArr.solutions[i];
        }
    }
    solution.vehicle1_solution.header.seq = the_seq;
    solution.vehicle2_solution.header.seq = the_seq;
    final_solution_arr.solution.push_back(solution);
    // Set new initial states:
    x1_1 = solution.vehicle1_solution.vehicle_states.x_1;
    x1_2 = solution.vehicle1_solution.vehicle_states.x_2;
    x1_3 = solution.vehicle1_solution.vehicle_states.x_3;
    x1_4 = solution.vehicle1_solution.vehicle_states.x_4;
    x1_5 = solution.vehicle1_solution.vehicle_states.x_5;
    x1_6 = solution.vehicle1_solution.vehicle_states.x_6;
    x1_7 = solution.vehicle1_solution.vehicle_states.x_7;

    x2_1 = solution.vehicle2_solution.vehicle_states.x_1;
    x2_2 = solution.vehicle2_solution.vehicle_states.x_2;
    x2_3 = solution.vehicle2_solution.vehicle_states.x_3;
    x2_4 = solution.vehicle2_solution.vehicle_states.x_4;
    x2_5 = solution.vehicle2_solution.vehicle_states.x_5;
    x2_6 = solution.vehicle2_solution.vehicle_states.x_6;
    x2_7 = solution.vehicle2_solution.vehicle_states.x_7;

    // Initializing new set of control signals considering previous solutoins
    /*if (0.5 * solution.vehicle1_solution.vehicle_control_signals.u) => u1_min {
        u1_min = 0.5 * solution.vehicle1_solution.vehicle_control_signals.u;
    }
    
    if (1.5 * solution.vehicle1_solution.vehicle_control_signals.u) <= u1_max {
        u1_max = 1.5 * solution.vehicle1_solution.vehicle_control_signals.u;
    }

    if (0.5 * solution.vehicle2_solution.vehicle_control_signals.u) >= u2_min {
        u2_min = 0.5 * solution.vehicle2_solution.vehicle_control_signals.u;
    }
    
    if (1.5 * solution.vehicle2_solution.vehicle_control_signals.u) <= u2_max {
        u2_max = 1.5 * solution.vehicle2_solution.vehicle_control_signals.u;
    }



    if (0.5 * solution.vehicle1_solution.vehicle_control_signals.u) >= u1_min {
        u1_min = 0.5 * solution.vehicle1_solution.vehicle_control_signals.u;
    }
    
    if (1.5 * solution.vehicle1_solution.vehicle_control_signals.u) <= u1_max {
        u1_max = 1.5 * solution.vehicle1_solution.vehicle_control_signals.u;
    }

    if (0.5 * solution.vehicle2_solution.vehicle_control_signals.u) >= u2_min {
        u2_min = 0.5 * solution.vehicle2_solution.vehicle_control_signals.u;
    }
    
    if (1.5 * solution.vehicle2_solution.vehicle_control_signals.u) <= u2_max {
        u2_max = 1.5 * solution.vehicle2_solution.vehicle_control_signals.u;
    }

    u2_min = 0.5 * solution.vehicle2_solution.vehicle_control_signals.u;
    
    u2_max = 1.5 * solution.vehicle2_solution.vehicle_control_signals.u;

    w1_min = 0.5 * solution.vehicle1_solution.vehicle_control_signals.w;
    w2_min = 0.5 * solution.vehicle2_solution.vehicle_control_signals.w;
    w1_max = 1.5 * solution.vehicle1_solution.vehicle_control_signals.w;
    w2_max = 1.5 * solution.vehicle2_solution.vehicle_control_signals.w;
    */

    // initializing new random signal generator
    //the_seq = 0;
    vehicle1_solutionsArr.solutions.clear();
    vehicle2_solutionsArr.solutions.clear();
    solution_step = solution_step + 1;

    // Finishig process condition 1
    if (solution_step == max_step)
    {
        preccessEnd = true;
    }

    // Finishig process condition 2
    if (abs(x1_1 - x1_1_final) < threshold && abs(x1_2 - x1_2_final) < threshold && abs(x2_1 - x2_1_final) < threshold && abs(x2_2 - x2_2_final) < threshold)
    {
        preccessEnd = true;
        //ROS_INFO("############################### abs(x1_1 - x1_1_final) : [%f]", abs(x1_1 - x1_1_final));
    }
}

void calcCollisionRisk()
{
    //ROS_INFO("###############################solution.navigation_cost: [%f]", pow(solution_step / (solution_step - predicted_final_step), 2));
    //return pow(solution_step / (solution_step - predicted_final_step), 2) * (pow(s.x_1 - x1_final, 2) + pow(s.x_2 - x2_final, 2) + pow(s.x_3 - x3_final, 2) + pow(s.x_4 - x4_final, 2));
    //return (pow(s.x_1 - x1_final, 2) + pow(s.x_2 - x2_final, 2) + pow(s.x_3 - x3_final, 2) + pow(s.x_4 - x4_final, 2));
    float x11 = vehicle1_solutionsArr.solutions.back().vehicle_states.x_1;
    float x21 = vehicle1_solutionsArr.solutions.back().vehicle_states.x_2;
    float x12 = vehicle2_solutionsArr.solutions.back().vehicle_states.x_1;
    float x22 = vehicle2_solutionsArr.solutions.back().vehicle_states.x_2;
    vehicle1_solutionsArr.solutions.back().collision_risk = 1/pow(x11 - x12, 2) + pow(x21 - x22, 2);
    vehicle2_solutionsArr.solutions.back().collision_risk = 1/pow(x11 - x12, 2) + pow(x21 - x22, 2);
}

float calcNavigationCost(const model_msgs::VehicleStates s, float x1_final, float x2_final, float x3_final, float x4_final)
{
    //ROS_INFO("###############################solution.navigation_cost: [%f]", pow(solution_step / (solution_step - predicted_final_step), 2));
    //return pow(solution_step / (solution_step - predicted_final_step), 2) * (pow(s.x_1 - x1_final, 2) + pow(s.x_2 - x2_final, 2) + pow(s.x_3 - x3_final, 2) + pow(s.x_4 - x4_final, 2));
    //return (pow(s.x_1 - x1_final, 2) + pow(s.x_2 - x2_final, 2) + pow(s.x_3 - x3_final, 2) + pow(s.x_4 - x4_final, 2));
    return (pow(s.x_1 - x1_final, 2) + pow(s.x_2 - x2_final, 2));
}

void vehicle1OutputCallback(const model_msgs::VehicleModelOutput msg)
{
    solver_msgs::solutionHolder solution;
    solution.header = msg.header;
    vehicel2_output_seq = solution.header.seq;
    solution.vehicle_states = msg.vehicle_states;
    solution.vehicle_control_signals = msg.vehicle_control_signals;
    solution.fuel_cost = msg.fuel_cost;
    solution.navigation_cost = calcNavigationCost(msg.vehicle_states, x1_1_final, x1_2_final, x1_3_final, x1_4_final);
    //ROS_INFO("###############################solution.navigation_cost: [%f]", solution.navigation_cost);
    vehicle1_solutionsArr.solutions.push_back(solution);

    vehicle1_output_msg_received = true;
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void vehicle2OutputCallback(const model_msgs::VehicleModelOutput msg)
{
    solver_msgs::solutionHolder solution;
    solution.header = msg.header;
    solution.vehicle_states = msg.vehicle_states;
    solution.vehicle_control_signals = msg.vehicle_control_signals;
    solution.fuel_cost = msg.fuel_cost;
    solution.navigation_cost = calcNavigationCost(msg.vehicle_states, x2_1_final, x2_2_final, x2_3_final, x2_4_final);

    vehicle2_solutionsArr.solutions.push_back(solution);
    vehicel1_output_seq = solution.header.seq;
    vehicle2_output_msg_received = true;
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "model");
    ros::NodeHandle n;

    n.param<int>("initial_seed_size", initial_seed_size, 50);
    n.param<int>("predicted_final_step", predicted_final_step, 5000);
    n.param<float>("threshold", threshold, 15);
    n.param<float>("solver_time_step", solver_time_step, 0.1);
    n.param<int>("max_step", max_step, 5000);

    n.param<float>("u1_min", u1_min, -70);
    n.param<float>("u2_min", u2_min, -70);
    n.param<float>("u1_max", u1_max, 170);
    n.param<float>("u2_max", u2_max, 170);
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
    n.param<float>("x1_7_init", x1_7, 0);

    n.param<float>("x2_1_init", x2_1, 400);
    n.param<float>("x2_2_init", x2_2, 100);
    n.param<float>("x2_3_init", x2_3, 0);
    n.param<float>("x2_4_init", x2_4, 0);
    n.param<float>("x2_5_init", x2_5, 0);
    n.param<float>("x2_6_init", x2_6, 0);
    n.param<float>("x2_7_init", x2_7, 0);

    n.param<float>("x1_1_final", x1_1_final, 300);
    n.param<float>("x1_2_final", x1_2_final, -100);
    n.param<float>("x1_3_final", x1_3_final, 0);
    n.param<float>("x1_4_final", x1_4_final, 0);
    n.param<float>("x1_5_final", x1_5_final, 0);
    n.param<float>("x1_6_final", x1_6_final, 0);
    n.param<float>("x1_7_final", x1_7_final, 0);

    n.param<float>("x2_1_final", x2_1_final, 0);
    n.param<float>("x2_2_final", x2_2_final, 0);
    n.param<float>("x2_3_final", x2_3_final, 0);
    n.param<float>("x2_4_final", x2_4_final, 0);
    n.param<float>("x2_5_final", x2_5_final, 0);
    n.param<float>("x2_6_final", x2_6_final, 0);
    n.param<float>("x2_7_final", x2_7_final, 0);

    ros::Publisher vehicle1_input_pub = n.advertise<model_msgs::VehicleModelInput>("/vehicle1/input", 1000);
    ros::Publisher solution_pub = n.advertise<solver_msgs::finalSolutionArr>("solution", 1000);
    ros::Publisher vehicle2_input_pub = n.advertise<model_msgs::VehicleModelInput>("/vehicle2/input", 1000);
    ros::Subscriber vehicle1_output_sub = n.subscribe("vehicle1/output", 1000, vehicle1OutputCallback);
    ros::Subscriber vehicle2_output_sub = n.subscribe("vehicle2/output", 1000, vehicle2OutputCallback);

    srand(static_cast<unsigned>(time(0)));
    //vehicle1_output_msg_received = true;
    //vehicle2_output_msg_received = true;

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        //ROS_INFO("********************here[%d], [%d], [%d]", vehicel1_output_seq, vehicel2_output_seq, the_seq);
        
        if (vehicel1_output_seq == vehicel2_output_seq)
        {
            if (vehicel1_output_seq > 0) {
                calcCollisionRisk();
            }
            
            ROS_INFO("********************Solver sent vehicle input sequence: [%d]", vehicel1_output_seq + 1);
            ros::Time time_stamp = ros::Time::now();

            //ROS_INFO("********************ve1 [%d]", initial_seed_size);
            model_msgs::VehicleModelInput vehicle_input;
            //the_seq = vehicel2_output_seq + 1;
            vehicle_input.header.seq = the_seq;
            vehicle_input.useThisStates = true;
            vehicle_input.solver_time_step = solver_time_step;

            vehicle_input.header.stamp = time_stamp;
            vehicle_input.vehicle_states.x_1 = x1_1;
            vehicle_input.vehicle_states.x_2 = x1_2;
            vehicle_input.vehicle_states.x_3 = x1_3;
            vehicle_input.vehicle_states.x_4 = x1_4;
            vehicle_input.vehicle_states.x_5 = x1_5;
            vehicle_input.vehicle_states.x_6 = x1_6;
            vehicle_input.vehicle_states.x_7 = x1_7;
            vehicle_input.vehicle_control_signals.u = u1_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (u1_max - u1_min)));
            vehicle_input.vehicle_control_signals.w = w1_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (w1_max - w1_min)));
            // TODO: Need to do for orientaion as well
            vehicle1_input_pub.publish(vehicle_input);

            vehicle_input.vehicle_states.x_1 = x2_1;
            vehicle_input.vehicle_states.x_2 = x2_2;
            vehicle_input.vehicle_states.x_3 = x2_3;
            vehicle_input.vehicle_states.x_4 = x2_4;
            vehicle_input.vehicle_states.x_5 = x2_5;
            vehicle_input.vehicle_states.x_6 = x2_6;
            vehicle_input.vehicle_states.x_7 = x2_7;
            vehicle_input.vehicle_control_signals.u = u2_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (u2_max - u2_min)));
            vehicle_input.vehicle_control_signals.w = w2_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (w2_max - w2_min)));
            // TODO: Need to do for orientaion as well
            vehicle2_input_pub.publish(vehicle_input);
            the_seq ++;

            //ROS_INFO("********************ve22");

            if (the_seq % initial_seed_size == 0)
            {
                evaluateSolutions();
                solution_pub.publish(final_solution_arr);
                //ROS_INFO("###############################HERE");
            }
        }

        ros::spinOnce();
        //ros::spin();
        loop_rate.sleep();
        if (preccessEnd)
        {
            break;
        }
    }
    return 0;
}