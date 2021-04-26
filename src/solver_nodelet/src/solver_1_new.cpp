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
#include "cav_vehicle_model_msgs/VehicleModelInput.h"
#include "cav_vehicle_model_msgs/VehicleModelOutput.h"
#include "cav_vehicle_model_msgs/VehicleStates.h"
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

  class cav_vehicle_model : public nodelet::Nodelet
  {
  public:
    cav_vehicle_model()
        : L(2.7),
          m(1200),
          // dt(0.5),
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

    cav_vehicle_model_msgs::VehicleModelOutput modelInputCallback(const cav_vehicle_model_msgs::VehicleModelInput msg, const double dt)
    {
      u = msg.vehicle_control_signals.u;
      w = msg.vehicle_control_signals.w;

      cav_vehicle_model_msgs::VehicleModelOutput output;
      output.header.seq = msg.header.seq;
      ROS_INFO("######################## Received message sequence: [%d]", msg.header.seq);

      // REF: https://www.coursera.org/lecture/intro-self-driving-cars/lesson-4-longitudinal-vehicle-modeling-V8htX
      float R_x = 0.1 * m * abs(msg.vehicle_states.x_4);   // Rolling resistan force N
      float F_aero = 0.1 * pow(msg.vehicle_states.x_4, 2); // Rolling resistan force N
      float F_load = F_aero + R_x;
      float je = Im_e + Im_t + Im_w * pow(GR, 2) + m * pow(GR, 2) * pow(R, 2);
      float dwheel_rot = (u - GR * R * F_load) / je;
      output.vehicle_states.x_5 = R * GR * dwheel_rot; //acceleration m/s^2
      output.vehicle_states.x_7 = msg.vehicle_states.x_7 + dt * w;        //front wheel angle
      output.vehicle_states.x_6 = atan(tan(x_7) / L);  // rad
      output.vehicle_states.x_4 = x_4 + dt * output.vehicle_states.x_5;
      output.vehicle_states.x_3 = x_3 + dt * (1.0 / L) * output.vehicle_states.x_4 * sin(output.vehicle_states.x_6); //rad
      output.vehicle_states.x_1 = x_1 + dt * output.vehicle_states.x_4 * cos(output.vehicle_states.x_3);
      output.vehicle_states.x_2 = x_2 + dt * output.vehicle_states.x_4 * sin(output.vehicle_states.x_3);

      output.vehicle_control_signals.u = u;
      output.vehicle_control_signals.w = w;
      output.fuel_cost = ideal_fuel_cost;
      if (u > 0) {output.fuel_cost = I_g * abs(x_4) * (B_1 + B_2 * I_g * abs(x_4) + B_3 * pow(I_g, 2) * pow(abs(x_4), 2)) + B_4 * u * abs(x_4) + B_5 * pow(abs(x_4), 2) * u * I_g + B_6 * abs(x_4) * pow(u, 2) / I_g;}
      return output;
    }

  private:
    float L, m, ga, dt, R, I_d, I_g, Im_e, Im_t, Im_w, GR, ideal_fuel_cost;
    float u, w;
    float x_1, x_2, x_3, x_4, x_5, x_6, x_7;
    float B_1, B_2, B_3, B_4, B_5, B_6;
    float rolling_res_force;

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
      // private_nh.getParam("dt", dt);                             // sampling time
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
      ouput_pub = private_nh.advertise<cav_vehicle_model_msgs::VehicleModelOutput>("output", 1000);
      nav_pub = private_nh.advertise<nav_msgs::Odometry>("odometry", 1000);
    }

    tf::TransformBroadcaster odom_broadcaster;
    ros::Publisher ouput_pub;
    ros::Publisher nav_pub;
    ros::Subscriber input_sub;
  };

  //
  //
  //
  //
  //
  //
  //
  //

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
          xA_1_final(-100),
          xA_2_final(100),
          xA_3_final(0),
          xA_4_final(0),
          x1_5_final(0),
          x1_6_final(0),
          x1_7_final(0),
          xB_1_final(0),
          xB_2_final(0),
          xB_3_final(0),
          xB_4_final(0),
          x2_5_final(0),
          x2_6_final(0),
          x2_7_final(0),
          u_initial(10.0),
          w_initial(0.0)
    {
    }

  private:
    bool proccessEnd = false;
    bool vehicleA_end = false;
    bool vehicleB_end = false;
    bool vehicel1_last_sent = false;
    bool vehicel2_last_sent = false;

    int the_seq = 0;

    int SOLVER_STEP = 0;
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
    float u_initial, w_initial;
    float x1_1, x1_2, x1_3, x1_4, x1_5, x1_6, x1_7;
    float x2_1, x2_2, x2_3, x2_4, x2_5, x2_6, x2_7;
    float xA_1_final, xA_2_final, xA_3_final, xA_4_final, x1_5_final, x1_6_final, x1_7_final;
    float xB_1_final, xB_2_final, xB_3_final, xB_4_final, x2_5_final, x2_6_final, x2_7_final;
    //float dx_1, dx_2, dx_3, dx_4, dx_5, dx_6;
    bool vehicleA_output_msg_received = false;
    bool vehicleB_output_msg_received = false;

    int initial_seed_size;
    solver_msgs::solutionHolderArr vehicleA_solutionsArr;
    solver_msgs::solutionHolderArr vehicleB_solutionsArr;
    solver_msgs::finalSolutionArr final_solution_arr;
    cav_vehicle_model _vehicleA_model, _vehicleB_model;
    struct solution_costs
    {
      float _colision_cost;
      float _navigation_cost;
      float _energy_cost;
    } _costs;
    
    std::vector<solution_costs> _cost_vector;

    void evaluateSolutions(const int _step, 
    const cav_vehicle_model_msgs::VehicleModelOutput _vehicleA_output,
    const cav_vehicle_model_msgs::VehicleModelOutput _vehicleB_output)
    {
      ROS_INFO("----------------- evalutaion START SOLVER_STEP: [%d]", _step);
      _costs._colision_cost = calcCollisionRisk(_vehicleA_output, _vehicleB_output);
      _costs._navigation_cost = calcNavigationCost(_vehicleA_output.vehicle_states, 
      xA_1_final, xA_2_final, xA_3_final, xA_4_final) + calcNavigationCost(_vehicleB_output.vehicle_states, 
      xB_1_final, xB_2_final, xB_3_final, xB_4_final);
      _costs._energy_cost = _vehicleA_output.fuel_cost + _vehicleB_output.fuel_cost;
      _cost_vector.push_back(_costs);
    }

    float calcCollisionRisk(const cav_vehicle_model_msgs::VehicleModelOutput _vehicleA_output,
    const cav_vehicle_model_msgs::VehicleModelOutput _vehicleB_output)
    {
      return 1 / pow((pow(abs(_vehicleA_output.vehicle_states.x_1 - _vehicleB_output.vehicle_states.x_1), 2) 
      + pow(abs(_vehicleA_output.vehicle_states.x_2 - _vehicleB_output.vehicle_states.x_2), 2)), 0.5);
    }

    float calcNavigationCost(const cav_vehicle_model_msgs::VehicleStates s, 
    float x1_final, float x2_final, float x3_final, float x4_final)
    {return (pow(abs(s.x_1 - x1_final), 2) + pow(abs(s.x_2 - x2_final), 2));}

    void vehicleAOutputCallback(const cav_vehicle_model_msgs::VehicleModelOutput msg)
    {
      if (!vehicleA_end)
      {
        solver_msgs::solutionHolder solution;
        solution.header = msg.header;
        //vehicel2_output_seq = solution.header.seq;
        solution.vehicle_states = msg.vehicle_states;
        solution.vehicle_control_signals = msg.vehicle_control_signals;
        solution.fuel_cost = msg.fuel_cost;
        solution.navigation_cost = calcNavigationCost(msg.vehicle_states, xA_1_final, xA_2_final, xA_3_final, xA_4_final);
        if (final_solution_arr.solution.size() > 0)
        {
          if (final_solution_arr.solution.back().vehicleA_solution.navigation_cost < threshold)
          {
            vehicleA_output_sub.shutdown();
            ROS_INFO("----------------- BINGO vehicleA: [%d]", SOLVER_STEP - 1);
            vehicleA_end = true;
            vehicel1_last_sent = true;
            //solution.vehicleA_solution.preccess_end = true;
          }
        }
        //ROS_INFO("###############################solution.navigation_cost: [%f]", solution.navigation_cost);
        vehicleA_solutionsArr.solutions.push_back(solution);
        vehicel1_output_seq = msg.header.seq;
        vehicleA_output_msg_received = true;
      }

      if (!vehicleA_end || !vehicleB_end)
      {
        ROS_INFO("********************generateInputs from vehicleAOutputCallback");
        generateInputs(SOLVER_STEP);
      }

      //ROS_INFO("I heard: [%s]", msg->data.c_str());
    }

    void vehicleBOutputCallback(const cav_vehicle_model_msgs::VehicleModelOutput msg)
    {
      if (!vehicleB_end)
      {
        solver_msgs::solutionHolder solution;
        solution.header = msg.header;
        solution.vehicle_states = msg.vehicle_states;
        solution.vehicle_control_signals = msg.vehicle_control_signals;
        solution.fuel_cost = msg.fuel_cost;
        solution.navigation_cost = calcNavigationCost(msg.vehicle_states, xB_1_final, xB_2_final, xB_3_final, xB_4_final);

        if (final_solution_arr.solution.size() > 0)
        {
          if (final_solution_arr.solution.back().vehicleB_solution.navigation_cost < threshold)
          {
            vehicleB_output_sub.shutdown();
            ROS_INFO("----------------- BINGO vehicleB: [%d]", SOLVER_STEP);
            vehicleB_end = true;
            vehicel2_last_sent = true;
            //solution.vehicleB_solution.preccess_end = true;
          }
        }

        vehicleB_solutionsArr.solutions.push_back(solution);
        //ROS_INFO("solution.header.seq: [%d]", solution.header.seq);
        vehicel2_output_seq = msg.header.seq;
        vehicleB_output_msg_received = true;
        //ROS_INFO("I heard: [%s]", msg->data.c_str());
      }

      if (!vehicleA_end || !vehicleB_end)
      {
        ROS_INFO("********************generateInputs from vehicleBOutputCallback");
        generateInputs(SOLVER_STEP);
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

      private_nh.getParam("xA_1_final", xA_1_final);
      private_nh.getParam("xA_2_final", xA_2_final);
      private_nh.getParam("xA_3_final", xA_3_final);
      private_nh.getParam("xA_4_final", xA_4_final);
      private_nh.getParam("x1_5_final", x1_5_final);
      private_nh.getParam("x1_6_final", x1_6_final);
      private_nh.getParam("x1_7_final", x1_7_final);

      private_nh.getParam("xB_1_final", xB_1_final);
      private_nh.getParam("xB_2_final", xB_2_final);
      private_nh.getParam("xB_3_final", xB_3_final);
      private_nh.getParam("xB_4_final", xB_4_final);
      private_nh.getParam("x2_5_final", x2_5_final);
      private_nh.getParam("x2_6_final", x2_6_final);
      private_nh.getParam("x2_7_final", x2_7_final);

      private_nh.getParam("u_initial", u_initial);
      private_nh.getParam("w_initial", w_initial);

      vehicleA_input_pub = private_nh.advertise<cav_vehicle_model_msgs::VehicleModelInput>("/cav_vehicle_model_1/input", 1000);
      solution_pub = private_nh.advertise<solver_msgs::finalSolutionArr>("solution", 1000);
      vehicleB_input_pub = private_nh.advertise<cav_vehicle_model_msgs::VehicleModelInput>("/cav_vehicle_model_2/input", 1000);
      vehicleA_output_sub = private_nh.subscribe("cav_vehicle_model_1/output", 1000, &solver_1::vehicleAOutputCallback, this);
      vehicleB_output_sub = private_nh.subscribe("cav_vehicle_model_2/output", 1000, &solver_1::vehicleBOutputCallback, this);

      //vehicleA_output_msg_received = true;
      //vehicleB_output_msg_received = true;
      srand(static_cast<unsigned>(time(0)));
      //ROS_INFO("*******************seqs [%d], [%d]", vehicel1_output_seq, vehicel2_output_seq);

      ROS_INFO("********************generateInputs from onInit");
      generateInputs(SOLVER_STEP);
    }

    ros::Publisher vehicleA_input_pub;
    ros::Publisher solution_pub;
    ros::Publisher vehicleB_input_pub;
    ros::Subscriber vehicleA_output_sub;
    ros::Subscriber vehicleB_output_sub;

    void generateInputs(const int _step)
    {
      // ros::Duration(0.00005).sleep();
      float _time_step_seconds = 1;
      cav_vehicle_model_msgs::VehicleModelInput _vehicleA_input, _vehicleB_input;
      _vehicleA_input.vehicle_states = _selected_solution.at(SOLVER_STEP).vehicleA.vehicle_states;
      _vehicleB_input.vehicle_states = _selected_solution.at(SOLVER_STEP).vehicleB.vehicle_states;
      if (_step == 0) {
        _vehicleA_input.vehicle_control_signals.u = u_initial;
        _vehicleA_input.vehicle_control_signals.w = w_initial;
      }

      ROS_INFO("********************START generateInputs at SOLVER_STEP: [%d]", SOLVER_STEP);
      evaluateSolutions(SOLVER_STEP, _vehicleA_model.modelInputCallback(_vehicleA_input, _time_step_seconds),
      _vehicleB_model.modelInputCallback(_vehicleB_input, _time_step_seconds));

      SOLVER_STEP++;
    }
  };

  PLUGINLIB_EXPORT_CLASS(solver_nodelet::solver_1, nodelet::Nodelet)
  PLUGINLIB_EXPORT_CLASS(solver_nodelet::cav_vehicle_model, nodelet::Nodelet)
} // namespace solver_nodelet
