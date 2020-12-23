/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "model_msgs/VehicleModelInput.h"
#include "model_msgs/VehicleModelOutput.h"
#include "model_msgs/VehicleStates.h"
#include "solver_msgs/solutionHolder.h"
#include "solver_msgs/solutionHolderArr.h"
#include "solver_msgs/finalSolutionArr.h"
#include "solver_msgs/finalSolution.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <math.h> //fabs
#include <stdio.h>

using namespace std;

namespace solver_nodelet
{

  class solver_1 : public nodelet::Nodelet
  {
  public:
    solver_1()
        : initial_seed_size(1000),
          predicted_final_step(50000),
          threshold(225),
          solver_time_step(0.5),
          max_step(50000),
          u1_min(-70),
          u2_min(-70),
          u1_max(170),
          u2_max(170),
          w1_min(-M_PI),
          w2_min(-M_PI),
          w1_max(M_PI),
          w2_max(M_PI),
          x1_1(0),
          x1_2(0),
          x1_3(0),
          x1_4(0),
          x1_5(0),
          x1_6(0),
          x1_7(0),
          x2_1(100),
          x2_2(-100),
          x2_3(0),
          x2_4(0),
          x2_5(0),
          x2_6(0),
          x2_7(0),
          x1_1_final(-100),
          x1_2_final(100),
          x1_3_final(0),
          x1_4_final(0),
          x1_5_final(0),
          x1_6_final(0),
          x1_7_final(0),
          x2_1_final(0),
          x2_2_final(0),
          x2_3_final(0),
          x2_4_final(0),
          x2_5_final(0),
          x2_6_final(0),
          x2_7_final(0)
    {
    }

  private:
    bool preccessEnd = false;
    bool vehicle1_end = false;
    bool vehicle2_end = false;
    bool vehicel1_last_sent = false;
    bool vehicel2_last_sent = false;

    int the_seq = 0;

    int solution_step = 1;
    int vehicel1_output_seq = 0;
    int vehicel2_output_seq = 0;
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

    void evaluateSolutions()
    {
      ROS_INFO("----------------- evalutaion START solution_step: [%d]", solution_step);
      solver_msgs::finalSolution solution;
      solution.solution_step = solution_step;
      //ROS_INFO("----------------- size: [%f]", vehicle1_solutionsArr.solutions[initial_seed_size/2].vehicle_control_signals.u);
      if (vehicle2_end)
      {
        solution.vehicle2_solution = final_solution_arr.solution.back().vehicle2_solution;
        solution.vehicle2_solution.preccess_end = true;
      }
      else
      {
        solution.vehicle2_solution = vehicle2_solutionsArr.solutions[initial_seed_size / 2];
      }

      if (vehicle1_end)
      {
        solution.vehicle1_solution = final_solution_arr.solution.back().vehicle1_solution;
        solution.vehicle1_solution.preccess_end = true;
      }
      else
      {
        solution.vehicle1_solution = vehicle1_solutionsArr.solutions[initial_seed_size / 2];
      }

      ROS_INFO("----------------- SHIT #1 evalutaion END solution_step: [%d]", solution_step);

      for (int i = 0; i < vehicle1_solutionsArr.solutions.size(); i++)
      {
        //if (vehicle1_solutionsArr.solutions[i].fuel_cost + vehicle2_solutionsArr.solutions[i].fuel_cost < solution.vehicle1_solution.fuel_cost + solution.vehicle2_solution.fuel_cost && vehicle1_solutionsArr.solutions[i].navigation_cost + vehicle2_solutionsArr.solutions[i].navigation_cost < solution.vehicle1_solution.navigation_cost + solution.vehicle2_solution.navigation_cost && vehicle1_solutionsArr.solutions[i].collision_risk + vehicle2_solutionsArr.solutions[i].collision_risk < solution.vehicle1_solution.collision_risk + solution.vehicle2_solution.collision_risk)
        if (vehicle1_solutionsArr.solutions[i].navigation_cost + vehicle2_solutionsArr.solutions[i].navigation_cost < solution.vehicle1_solution.navigation_cost + solution.vehicle2_solution.navigation_cost)
        //if (vehicle1_solutionsArr.solutions[i].fuel_cost + vehicle2_solutionsArr.solutions[i].fuel_cost < solution.vehicle1_solution.fuel_cost + solution.vehicle2_solution.fuel_cost && vehicle1_solutionsArr.solutions[i].navigation_cost + vehicle2_solutionsArr.solutions[i].navigation_cost < solution.vehicle1_solution.navigation_cost + solution.vehicle2_solution.navigation_cost)
        //if (vehicle1_solutionsArr.solutions[i].navigation_cost + vehicle2_solutionsArr.solutions[i].navigation_cost < solution.vehicle1_solution.navigation_cost + solution.vehicle2_solution.navigation_cost)
        {
          solution.vehicle1_solution = vehicle1_solutionsArr.solutions[i];
          solution.vehicle2_solution = vehicle2_solutionsArr.solutions[i];
        }
      }
      ROS_INFO("----------------- SHIT #2 evalutaion END solution_step: [%d]", solution_step);
      solution.vehicle1_solution.header.seq = the_seq;
      solution.vehicle2_solution.header.seq = the_seq;
      ROS_INFO("----------------- SHIT #2,5 evalutaion END the_seq: [%d]", the_seq);
      //ROS_INFO("----------------- SHIT #2,5 evalutaion END the_seq: [%f]", final_solution_arr.solution.back().vehicle2_solution.navigation_cost);

      final_solution_arr.solution.push_back(solution);
      ROS_INFO("----------------- SHIT #4 evalutaion END solution_step: [%d]", solution_step);
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
      vehicle1_solutionsArr.solutions.clear();
      vehicle2_solutionsArr.solutions.clear();
      ROS_INFO("----------------- evalutaion END solution_step: [%d]", solution_step);
      solution_step = solution_step + 1;

      if (vehicle2_end && vehicle1_end)
      {
        ROS_INFO("----------------- BINGO BOTH: [%d]", solution_step);
        preccessEnd = true;
      }
    }

    void calcCollisionRisk()
    {
      //ROS_INFO("###############################solution.navigation_cost: [%f]", pow(solution_step / (solution_step - predicted_final_step), 2));
      //return pow(solution_step / (solution_step - predicted_final_step), 2) * (pow(s.x_1 - x1_final, 2) + pow(s.x_2 - x2_final, 2) + pow(s.x_3 - x3_final, 2) + pow(s.x_4 - x4_final, 2));
      //return (pow(s.x_1 - x1_final, 2) + pow(s.x_2 - x2_final, 2) + pow(s.x_3 - x3_final, 2) + pow(s.x_4 - x4_final, 2));
      //ROS_INFO("*******************vehicle1_solutionsArr [%f]", vehicle1_solutionsArr.solutions.back().vehicle_states.x_1);
      float x12, x22, x11, x21;
      if (vehicle2_solutionsArr.solutions.size() > 0)
      {
        x12 = vehicle2_solutionsArr.solutions.back().vehicle_states.x_1;
        x22 = vehicle2_solutionsArr.solutions.back().vehicle_states.x_2;
        vehicle2_solutionsArr.solutions.back().collision_risk = 1 / pow((pow(abs(x11 - x12), 2) + pow(abs(x21 - x22), 2)), 0.5);
      }
      else
      {
        //x12 = final_solution_arr.solution.back().vehicle2_solution.vehicle_states.x_1;
        //x22 = final_solution_arr.solution.back().vehicle2_solution.vehicle_states.x_2;
      }
      if (vehicle1_solutionsArr.solutions.size() > 0)
      {
        x11 = vehicle1_solutionsArr.solutions.back().vehicle_states.x_1;
        x21 = vehicle1_solutionsArr.solutions.back().vehicle_states.x_2;
        vehicle1_solutionsArr.solutions.back().collision_risk = 1 / pow((pow(abs(x11 - x12), 2) + pow(abs(x21 - x22), 2)), 0.5);
      }
      else
      {
        //x11 = final_solution_arr.solution.back().vehicle1_solution.vehicle_states.x_1;
        //x21 = final_solution_arr.solution.back().vehicle1_solution.vehicle_states.x_2;
      }

      //ROS_INFO("*******************seqs [%d], [%d]", vehicel1_output_seq, vehicel2_output_seq);
      
      
      //ROS_INFO("*******************seqs [%d], [%d]", vehicel1_output_seq, vehicel2_output_seq);
    }

    float calcNavigationCost(const model_msgs::VehicleStates s, float x1_final, float x2_final, float x3_final, float x4_final)
    {
      //ROS_INFO("###############################solution.navigation_cost: [%f]", pow(solution_step / (solution_step - predicted_final_step), 2));
      //return pow(solution_step / (solution_step - predicted_final_step), 2) * (pow(s.x_1 - x1_final, 2) + pow(s.x_2 - x2_final, 2) + pow(s.x_3 - x3_final, 2) + pow(s.x_4 - x4_final, 2));
      //return (pow(s.x_1 - x1_final, 2) + pow(s.x_2 - x2_final, 2) + pow(s.x_3 - x3_final, 2) + pow(s.x_4 - x4_final, 2));
      //return (pow(abs(s.x_1 - x1_final), 2) + pow(abs(s.x_2 - x2_final), 2) + pow(abs(s.x_3 - x3_final), 2) + pow(abs(s.x_4 - x4_final), 2));
      return (pow(abs(s.x_1 - x1_final), 2) + pow(abs(s.x_2 - x2_final), 2));
    }

    void vehicle1OutputCallback(const model_msgs::VehicleModelOutput msg)
    {
      if (!vehicle1_end)
      {
        solver_msgs::solutionHolder solution;
        solution.header = msg.header;
        //vehicel2_output_seq = solution.header.seq;
        solution.vehicle_states = msg.vehicle_states;
        solution.vehicle_control_signals = msg.vehicle_control_signals;
        solution.fuel_cost = msg.fuel_cost;
        solution.navigation_cost = calcNavigationCost(msg.vehicle_states, x1_1_final, x1_2_final, x1_3_final, x1_4_final);
        if (final_solution_arr.solution.size() > 0)
        {
          if (final_solution_arr.solution.back().vehicle1_solution.navigation_cost < threshold)
          {
            vehicle1_output_sub.shutdown();
            ROS_INFO("----------------- BINGO vehicle1: [%d]", solution_step - 1);
            vehicle1_end = true;
            vehicel1_last_sent = true;
            //solution.vehicle1_solution.preccess_end = true;
          }
        }
        //ROS_INFO("###############################solution.navigation_cost: [%f]", solution.navigation_cost);
        vehicle1_solutionsArr.solutions.push_back(solution);
        vehicel1_output_seq = msg.header.seq;
        vehicle1_output_msg_received = true;
      }

      if (!vehicle1_end || !vehicle2_end)
      {
        ROS_INFO("********************generateInputs from vehicle1OutputCallback");
        generateInputs();
      }

      //ROS_INFO("I heard: [%s]", msg->data.c_str());
    }

    void vehicle2OutputCallback(const model_msgs::VehicleModelOutput msg)
    {
      if (!vehicle2_end)
      {
        solver_msgs::solutionHolder solution;
        solution.header = msg.header;
        solution.vehicle_states = msg.vehicle_states;
        solution.vehicle_control_signals = msg.vehicle_control_signals;
        solution.fuel_cost = msg.fuel_cost;
        solution.navigation_cost = calcNavigationCost(msg.vehicle_states, x2_1_final, x2_2_final, x2_3_final, x2_4_final);

        if (final_solution_arr.solution.size() > 0)
        {
          if (final_solution_arr.solution.back().vehicle2_solution.navigation_cost < threshold)
          {
            vehicle2_output_sub.shutdown();
            ROS_INFO("----------------- BINGO vehicle2: [%d]", solution_step);
            vehicle2_end = true;
            vehicel2_last_sent = true;
            //solution.vehicle2_solution.preccess_end = true;
          }
        }

        vehicle2_solutionsArr.solutions.push_back(solution);
        //ROS_INFO("solution.header.seq: [%d]", solution.header.seq);
        vehicel2_output_seq = msg.header.seq;
        vehicle2_output_msg_received = true;
        //ROS_INFO("I heard: [%s]", msg->data.c_str());
      }

      if (!vehicle1_end || !vehicle2_end)
      {
        ROS_INFO("********************generateInputs from vehicle2OutputCallback");
        generateInputs();
      }
    }

    virtual void onInit()
    {
      ros::NodeHandle &private_nh = getPrivateNodeHandle();

      private_nh.getParam("initial_seed_size", initial_seed_size);
      private_nh.getParam("predicted_final_step", predicted_final_step);
      private_nh.getParam("threshold", threshold);
      private_nh.getParam("solver_time_step", solver_time_step);
      private_nh.getParam("max_step", max_step);

      private_nh.getParam("u1_min", u1_min);
      private_nh.getParam("u2_min", u2_min);
      private_nh.getParam("u1_max", u1_max);
      private_nh.getParam("u2_max", u2_max);
      private_nh.getParam("w1_min", w1_min);
      private_nh.getParam("w2_min", w2_min);
      private_nh.getParam("w1_max", w1_max);
      private_nh.getParam("w2_max", w2_max);

      private_nh.getParam("x1_1_init", x1_1);
      private_nh.getParam("x1_2_init", x1_2);
      private_nh.getParam("x1_3_init", x1_3);
      private_nh.getParam("x1_4_init", x1_4);
      private_nh.getParam("x1_5_init", x1_5);
      private_nh.getParam("x1_6_init", x1_6);
      private_nh.getParam("x1_7_init", x1_7);

      private_nh.getParam("x2_1_init", x2_1);
      private_nh.getParam("x2_2_init", x2_2);
      private_nh.getParam("x2_3_init", x2_3);
      private_nh.getParam("x2_4_init", x2_4);
      private_nh.getParam("x2_5_init", x2_5);
      private_nh.getParam("x2_6_init", x2_6);
      private_nh.getParam("x2_7_init", x2_7);

      private_nh.getParam("x1_1_final", x1_1_final);
      private_nh.getParam("x1_2_final", x1_2_final);
      private_nh.getParam("x1_3_final", x1_3_final);
      private_nh.getParam("x1_4_final", x1_4_final);
      private_nh.getParam("x1_5_final", x1_5_final);
      private_nh.getParam("x1_6_final", x1_6_final);
      private_nh.getParam("x1_7_final", x1_7_final);

      private_nh.getParam("x2_1_final", x2_1_final);
      private_nh.getParam("x2_2_final", x2_2_final);
      private_nh.getParam("x2_3_final", x2_3_final);
      private_nh.getParam("x2_4_final", x2_4_final);
      private_nh.getParam("x2_5_final", x2_5_final);
      private_nh.getParam("x2_6_final", x2_6_final);
      private_nh.getParam("x2_7_final", x2_7_final);

      vehicle1_input_pub = private_nh.advertise<model_msgs::VehicleModelInput>("/vehicle_model_1/input", 1000);
      solution_pub = private_nh.advertise<solver_msgs::finalSolutionArr>("solution", 1000);
      vehicle2_input_pub = private_nh.advertise<model_msgs::VehicleModelInput>("/vehicle_model_2/input", 1000);
      vehicle1_output_sub = private_nh.subscribe("vehicle_model_1/output", 1000, &solver_1::vehicle1OutputCallback, this);
      vehicle2_output_sub = private_nh.subscribe("vehicle_model_2/output", 1000, &solver_1::vehicle2OutputCallback, this);

      //vehicle1_output_msg_received = true;
      //vehicle2_output_msg_received = true;
      srand(static_cast<unsigned>(time(0)));
      //ROS_INFO("*******************seqs [%d], [%d]", vehicel1_output_seq, vehicel2_output_seq);

      ROS_INFO("********************generateInputs from onInit");
      generateInputs();
    }
    ros::Publisher vehicle1_input_pub;
    ros::Publisher solution_pub;
    ros::Publisher vehicle2_input_pub;
    ros::Subscriber vehicle1_output_sub;
    ros::Subscriber vehicle2_output_sub;

    void generateInputs()
    {
      ros::Duration(0.00005).sleep();
      ROS_INFO("********************START generateInputs: [%d], [%d]", vehicel1_output_seq, vehicel2_output_seq);
      if ((vehicel1_output_seq == vehicel2_output_seq || vehicle2_end || vehicle1_end) && (vehicel1_output_seq != -1 && vehicel2_output_seq != -1))
      {
        if ((vehicel1_output_seq > 1 && vehicel2_output_seq > 1))
        {
          ROS_INFO("********************START calcCollisionRisk");
          calcCollisionRisk();
          ROS_INFO("********************END calcCollisionRisk");
        }

        ROS_INFO("********************Solver sent vehicle input sequence: [%d]", the_seq);
        //ros::Time time_stamp = ros::Time::now();

        //ROS_INFO("********************ve1 [%d]", initial_seed_size);
        //if (!vehicle1_end || vehicel1_last_sent)
        if (!vehicle1_end)
        {
          //vehicel1_last_sent = false;
          model_msgs::VehicleModelInput vehicle1_input;
          //the_seq = vehicel2_output_seq + 1;
          vehicle1_input.header.seq = the_seq;
          vehicle1_input.header.stamp = ros::Time::now();
          vehicle1_input.useThisStates = true;
          vehicle1_input.solver_time_step = solver_time_step;

          vehicle1_input.vehicle_states.x_1 = x1_1;
          vehicle1_input.vehicle_states.x_2 = x1_2;
          vehicle1_input.vehicle_states.x_3 = x1_3;
          vehicle1_input.vehicle_states.x_4 = x1_4;
          vehicle1_input.vehicle_states.x_5 = x1_5;
          vehicle1_input.vehicle_states.x_6 = x1_6;
          vehicle1_input.vehicle_states.x_7 = x1_7;
          vehicle1_input.vehicle_control_signals.u = u1_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (u1_max - u1_min)));
          vehicle1_input.vehicle_control_signals.w = w1_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (w1_max - w1_min)));
          // TODO: Need to do for orientaion as well
          vehicle1_input_pub.publish(vehicle1_input);
        }

        //if (!vehicle2_end || vehicel2_last_sent)
        if (!vehicle2_end)
        {
          //vehicel2_last_sent = false;
          model_msgs::VehicleModelInput vehicle2_input;
          //the_seq = vehicel2_output_seq + 1;
          vehicle2_input.header.seq = the_seq;
          vehicle2_input.header.stamp = ros::Time::now();
          vehicle2_input.useThisStates = true;
          vehicle2_input.solver_time_step = solver_time_step;

          vehicle2_input.vehicle_states.x_1 = x2_1;
          vehicle2_input.vehicle_states.x_2 = x2_2;
          vehicle2_input.vehicle_states.x_3 = x2_3;
          vehicle2_input.vehicle_states.x_4 = x2_4;
          vehicle2_input.vehicle_states.x_5 = x2_5;
          vehicle2_input.vehicle_states.x_6 = x2_6;
          vehicle2_input.vehicle_states.x_7 = x2_7;
          vehicle2_input.vehicle_control_signals.u = u2_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (u2_max - u2_min)));
          vehicle2_input.vehicle_control_signals.w = w2_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (w2_max - w2_min)));
          // TODO: Need to do for orientaion as well
          vehicle2_input_pub.publish(vehicle2_input);
        }

        the_seq++;

        if (the_seq % initial_seed_size == 0)
        {
          //the_seq = 1;
          ros::Duration(0.1).sleep();
          evaluateSolutions();
          if (!preccessEnd)
          {
            //ros::Duration(0.1).sleep();
            solution_pub.publish(final_solution_arr);
            ROS_INFO("********************final_solution_arr published");
            //ros::Duration(0.1).sleep();
          }
          else
          {
            ROS_INFO("********************preccessEnd");
          }
          //
        }
      }
      else
      {
        ROS_INFO("********************generateInput NOT HAPPENDS!");
      }

      //SROS_INFO("********************ve22");
    }
  };
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  //
  class vehicle_model : public nodelet::Nodelet
  {
  public:
    vehicle_model()
        : L(2.7),
          m(1200),
          dt(0.5),
          R(0.19),
          I_d(0.3),
          I_g(0.3),
          Im_e(0.3),
          Im_t(0.3),
          Im_w(0.3),
          solver_time_step(0.5),
          solver_req(true),
          ideal_fuel_cost(0.01)
    {
    }

  private:
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
    virtual void onInit()
    {
      ros::NodeHandle &private_nh = getPrivateNodeHandle();
      //private_nh.getParam("value", value_);
      private_nh.getParam("L", L);                               //wheelbase
      private_nh.getParam("m", m);                               // mass
      private_nh.getParam("dt", dt);                             // sampling time
      private_nh.getParam("R", R);                               //  effective redius
      private_nh.getParam("I_d", I_d);                           // differential ratio
      private_nh.getParam("I_g", I_g);                           // gear ratio
      private_nh.getParam("Im_e", Im_e);                         // engine rotation momentume
      private_nh.getParam("Im_t", Im_t);                         // turbine rotation momentume
      private_nh.getParam("Im_w", Im_w);                         // wheel rotation momentume
      private_nh.getParam("solver_time_step", solver_time_step); // wheel rotation momentume
      private_nh.getParam("solver_req", solver_req);             // wheel rotation momentume
      private_nh.getParam("ideal_fuel_cost", ideal_fuel_cost);   // wheel rotation momentume

      //private_nh.getParam("GR", GR, 0.3);
      GR = I_d * I_g;
      B_1 = 1.1046 * 0.01 * I_d / R;
      B_2 = -7.7511 * (pow(10, -5)) * (pow(I_d, 2)) / pow(R, 2);
      B_3 = 1.6958 * (pow(10, -7)) * (pow(I_d, 3)) / pow(R, 3);
      B_4 = 1.7363 * (pow(10, -5)) / R;
      B_5 = 6.4277 * (pow(10, -8)) * I_d / pow(R, 2);
      B_6 = 1.6088 * (pow(10, -7)) / (R * I_d);
      ouput_pub = private_nh.advertise<model_msgs::VehicleModelOutput>("output", 1000);
      nav_pub = private_nh.advertise<nav_msgs::Odometry>("odometry", 1000);
      input_sub = private_nh.subscribe("input", 1000, &vehicle_model::inputCallback, this);

      if (solver_req)
      {
        dt = solver_time_step;
        //ROS_INFO("########################y:");
      }
    }

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
      if (u <= 0)
      {
        output.fuel_cost = ideal_fuel_cost;
      }
      else
      {
        output.fuel_cost = I_g * abs(x_4) * (B_1 + B_2 * I_g * abs(x_4) + B_3 * pow(I_g, 2) * pow(abs(x_4), 2)) + B_4 * u * abs(x_4) + B_5 * pow(abs(x_4), 2) * u * I_g + B_6 * abs(x_4) * pow(u, 2) / I_g;
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
      //ROS_INFO("I heard: [%s]", msg->data.c_str());
    }
    tf::TransformBroadcaster odom_broadcaster;
    ros::Publisher ouput_pub;
    ros::Publisher nav_pub;
    ros::Subscriber input_sub;
  };

  PLUGINLIB_EXPORT_CLASS(solver_nodelet::solver_1, nodelet::Nodelet)
  PLUGINLIB_EXPORT_CLASS(solver_nodelet::vehicle_model, nodelet::Nodelet)
} // namespace solver_nodelet
