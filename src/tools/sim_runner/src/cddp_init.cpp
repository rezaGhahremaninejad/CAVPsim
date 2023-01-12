#include "ros/ros.h"
#include <ctime>
#include <cmath>
#include <iostream>
#include <cstdio>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "autoware_msgs/LaneArray.h"
#include <algorithm>
#include <random>
#include <vector>
#include <cstdlib>
#include <numeric>


struct position {
    float x;
    float y;
    float a;
};
position Cu1, Cu2, Cu3, Cu4, Cu5, Cu6, Go1, Go2, Go3, Go4, Go5, Go6;
std::vector<position> curr_pos, goal_pos;

std::string NS;
ros::Publisher goal_pub;

void initTHIS(void) {
    Cu1.x = -1.63;
    Cu1.y = 0.377;
    Cu1.a = 0.079;
    curr_pos.push_back(Cu1);

    Cu2.x = 30.056;
    Cu2.y = 2.241;
    Cu2.a = 0.153;
    curr_pos.push_back(Cu2);

    Cu3.x = 63.215;
    Cu3.y = 7.035;
    Cu3.a = 0.111;
    curr_pos.push_back(Cu3);

    Cu4.x = 109.157;
    Cu4.y = -23.46;
    Cu4.a = -1.459;
    curr_pos.push_back(Cu4);

    Cu5.x = 114.35;
    Cu5.y = -91.907;
    Cu5.a = -1.491;
    curr_pos.push_back(Cu5);

    Cu6.x = 119.81;
    Cu6.y = -177.0;
    Cu6.a = -1.518;
    curr_pos.push_back(Cu6);

    Go1.x = 121.874;
    Go1.y = -218.535;
    Go1.a = -1.521;
    goal_pos.push_back(Go1);

    Go2.x = 107.369;
    Go2.y = -270.137;
    Go2.a = -3.070;
    goal_pos.push_back(Go2);

    Go3.x = 62.544;
    Go3.y = -263.835;
    Go3.a = 1.614;
    goal_pos.push_back(Go3);

    Go4.x = 59.453;
    Go4.y = -211.163;
    Go4.a = 1.657;
    goal_pos.push_back(Go4);

    Go5.x = 55.529;
    Go5.y = -173.116;
    Go5.a = 1.654;
    goal_pos.push_back(Go5);

    Go6.x = 54.03;
    Go6.y = -123.869;
    Go6.a = 1.659;
    goal_pos.push_back(Go6);
}

// template<typename T>
// T random(std::vector<T> const &v)
// {
//     int r = rand() % v.size();
//     return v[r];
// }

// geometry_msgs::PoseStamped selectCurrPose() {
//     geometry_msgs::PoseStamped _res;
//     // int random = 0;
//     // int random = curr_pos.size()*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
//     position _p = random(curr_pos);
//     // std::cout << NS << "---CURR_p: " <<  _p.x << std::endl;  
//     _res.pose.position.x = _p.x;
//     _res.pose.position.y = _p.y;
//     tf2::Quaternion q;
//     q.setRPY(0, 0, _p.a);
//     _res.pose.orientation = tf2::toMsg(q);
//     return _res;
// }

// geometry_msgs::PoseStamped selectGoalPose() {
//     geometry_msgs::PoseStamped _res;
//     // int random = goal_pos.size()*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
//     // std::cout << NS << "---GOAL random: " <<  random << std::endl;  
//     // position _p = goal_pos.at(random);
//     position _p = random(goal_pos);
//     // std::cout << NS << "---GOAL_p: " <<  _p.x << std::endl;  
//     _res.pose.position.x = _p.x;
//     _res.pose.position.y = _p.y;
//     tf2::Quaternion q;
//     q.setRPY(0, 0, _p.a);
//     _res.pose.orientation = tf2::toMsg(q);
//     return _res;
// }

void egoPathCallback(const autoware_msgs::LaneArray) {
    goal_pub.shutdown();
}

int main(int argc, char **argv)
{
    // std::cout << "---------------NS: " << NS << std::endl;
    ros::init(argc, argv, "cddp_init");
    ros::NodeHandle n;
    NS = argv[argc-1];
    ros::Publisher initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
    ros::Publisher currrent_pose_pub = n.advertise<geometry_msgs::PoseStamped>("current_pose", 1000);
    ros::Publisher currrent_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("current_velocity", 1000);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
    ros::Subscriber ego_path_sub = n.subscribe("/lane_waypoints_array", 1, egoPathCallback);

    
    ros::Rate loop_rate(10);
    initTHIS();
    geometry_msgs::PoseWithCovarianceStamped _init_pose;
    geometry_msgs::PoseStamped _curr_pose, _goal;
    geometry_msgs::TwistStamped _curr_velocity;

    // _curr_pose = selectCurrPose();
    // _goal = selectGoalPose();

    while (ros::ok())
    {
        if (NS == "v_1") {
            _curr_pose.pose.position.x = curr_pos[0].x;
            _curr_pose.pose.position.y = curr_pos[0].y;
            tf2::Quaternion q;
            q.setRPY(0, 0, curr_pos[0].a);
            _curr_pose.pose.orientation = tf2::toMsg(q);
        } else if (NS == "v_2") {
            _curr_pose.pose.position.x = curr_pos[4].x;
            _curr_pose.pose.position.y = curr_pos[4].y;
            tf2::Quaternion q;
            q.setRPY(0, 0, curr_pos[4].a);
            _curr_pose.pose.orientation = tf2::toMsg(q);
        } else if (NS == "v_3") {
            _curr_pose.pose.position.x = curr_pos[2].x;
            _curr_pose.pose.position.y = curr_pos[2].y;
            tf2::Quaternion q;
            q.setRPY(0, 0, curr_pos[2].a);
            _curr_pose.pose.orientation = tf2::toMsg(q);
        } else if (NS == "v_4") {
            _curr_pose.pose.position.x = curr_pos[3].x;
            _curr_pose.pose.position.y = curr_pos[3].y;
            tf2::Quaternion q;
            q.setRPY(0, 0, curr_pos[3].a);
            _curr_pose.pose.orientation = tf2::toMsg(q);
        } else if (NS == "v_5") {
            _curr_pose.pose.position.x = curr_pos[4].x;
            _curr_pose.pose.position.y = curr_pos[4].y;
            tf2::Quaternion q;
            q.setRPY(0, 0, curr_pos[4].a);
            _curr_pose.pose.orientation = tf2::toMsg(q);
        } else if (NS == "v_6") {
            _curr_pose.pose.position.x = curr_pos[5].x;
            _curr_pose.pose.position.y = curr_pos[5].y;
            tf2::Quaternion q;
            q.setRPY(0, 0, curr_pos[5].a);
            _curr_pose.pose.orientation = tf2::toMsg(q);
        }

        if (NS == "v_1") {
            _goal.pose.position.x = goal_pos[0].x;
            _goal.pose.position.y = goal_pos[0].y;
            tf2::Quaternion q;
            q.setRPY(0, 0, goal_pos[0].a);
            _goal.pose.orientation = tf2::toMsg(q);
        } else if (NS == "v_2") {
            _goal.pose.position.x = goal_pos[4].x;
            _goal.pose.position.y = goal_pos[4].y;
            tf2::Quaternion q;
            q.setRPY(0, 0, goal_pos[4].a);
            _goal.pose.orientation = tf2::toMsg(q);
        } else if (NS == "v_3") {
            _goal.pose.position.x = goal_pos[2].x;
            _goal.pose.position.y = goal_pos[2].y;
            tf2::Quaternion q;
            q.setRPY(0, 0, goal_pos[2].a);
            _goal.pose.orientation = tf2::toMsg(q);
        } else if (NS == "v_4") {
            _goal.pose.position.x = goal_pos[3].x;
            _goal.pose.position.y = goal_pos[3].y;
            tf2::Quaternion q;
            q.setRPY(0, 0, goal_pos[3].a);
            _goal.pose.orientation = tf2::toMsg(q);
        } else if (NS == "v_5") {
            _goal.pose.position.x = goal_pos[4].x;
            _goal.pose.position.y = goal_pos[4].y;
            tf2::Quaternion q;
            q.setRPY(0, 0, goal_pos[4].a);
            _goal.pose.orientation = tf2::toMsg(q);
        } else if (NS == "v_6") {
            _goal.pose.position.x = goal_pos[5].x;
            _goal.pose.position.y = goal_pos[5].y;
            tf2::Quaternion q;
            q.setRPY(0, 0, goal_pos[5].a);
            _goal.pose.orientation = tf2::toMsg(q);
        }

        if (currrent_pose_pub.getNumSubscribers() > 0) {
            currrent_pose_pub.publish(_curr_pose);
            // currrent_pose_pub.shutdown();
        }

        if (goal_pub.getNumSubscribers() > 0) {
            goal_pub.publish(_goal);
            // goal_pub.shutdown();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}