#include "ros/ros.h"
#include "computation_msgs/status.h"
#include "communication_msgs/ComMessage.h"
#include "rosgraph_msgs/Clock.h"
#include <ctime>
#include <cmath>

double batch_size, t_op, app_flo;
float MIN_POINTS_DIST = 0.1;
float SAFE_DISTANCE = 6; //meters
float eg_col_dist;
ros::Publisher updated_lane_pub;
rosgraph_msgs::Clock _clock;
autoware_msgs::LaneArray updated_lane_waypoints;

int seq;
// #define LOOP_REPS 4294967295
int LOOP_REPS;
bool GOT_FIRST_MSG = false;
using namespace std;

cooperative_msgs::status cooperation_status;
autoware_msgs::LaneArray ego_path;

void rxCallback(const communication_msgs::ComMessage::ConstPtr &msg)
{
    float subject_col_dist = 0;
    if (!GOT_FIRST_MSG) {
        if (msg->ego_path.lanes.size() > 0 && ego_path.lanes.size() > 0) {            
            for (int i = 0; i < msg->ego_path.lanes[0].waypoints.size(); i++) {
                eg_col_dist = 0;
                if (i != msg->ego_path.lanes[0].waypoints.size() - 1 ) 
                {
                    subject_col_dist = subject_col_dist + pow(pow(msg->ego_path.lanes[0].waypoints[i].pose.pose.position.x 
                    - msg->ego_path.lanes[0].waypoints[i+1].pose.pose.position.x, 2) 
                    + pow(msg->ego_path.lanes[0].waypoints[i].pose.pose.position.y 
                    - msg->ego_path.lanes[0].waypoints[i+1].pose.pose.position.y, 2), 0.5);
                }  
                for (int j = 0; j < ego_path.lanes[0].waypoints.size(); j++) {
                    // std::cout << "----------------index of ego vehicle path j: " << std::to_string(i) << std::endl; 
                    if (j != msg->ego_path.lanes[0].waypoints.size() - 1 ) 
                    {
                        eg_col_dist = eg_col_dist + pow(pow(ego_path.lanes[0].waypoints[j].pose.pose.position.x 
                        - ego_path.lanes[0].waypoints[j+1].pose.pose.position.x, 2) 
                        + pow(ego_path.lanes[0].waypoints[j].pose.pose.position.y 
                        - ego_path.lanes[0].waypoints[j+1].pose.pose.position.y, 2), 0.5);
                    } 
                    if (pow(pow(msg->ego_path.lanes[0].waypoints[i].pose.pose.position.x 
                    - ego_path.lanes[0].waypoints[j].pose.pose.position.x, 2) 
                    + pow(msg->ego_path.lanes[0].waypoints[i].pose.pose.position.y 
                    - ego_path.lanes[0].waypoints[j].pose.pose.position.y, 2), 0.5) < MIN_POINTS_DIST) {
                        cooperation_status.COLLISSION_DETECT = true;
                        cooperation_status.COLLISION_DISTANC = eg_col_dist;
                        cooperation_status.COLLISION_TIME = eg_col_dist/ego_path.lanes[0].waypoints[j].twist.twist.linear.x;
                        if (msg->ego_status.COLLISSION_DETECT) {
                            if (cooperation_status.COLLISION_TIME > msg->ego_status.COLLISION_TIME ) 
                            {
                                cooperation_status.LAST_DECISION = cooperative_msgs::status::UPDATING_PATH;
                                updated_lane_waypoints = ego_path;
                                
                                for (int k = 0; k < j; k++) {
                                    // std::cout << "-------------------------k:" << std::to_string(k) << std::endl;
                                    updated_lane_waypoints.lanes[0].waypoints[k].twist.twist.linear.x = (eg_col_dist - SAFE_DISTANCE)/cooperation_status.COLLISION_TIME;
                                }
                            }
                        }
                        
                        i = msg->ego_path.lanes[0].waypoints.size();
                        j = ego_path.lanes[0].waypoints.size();
                    }
                } 
            }
        } else {std::cout << "------msg->ego_path.lanes.size(): " << std::to_string(msg->ego_path.lanes.size()) << std::endl;}
    }
}

void egoPathCallback(const autoware_msgs::LaneArray::ConstPtr &msg) {
    ego_path = *msg;    
}


int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "compute_model");
    ros::NodeHandle n;
    n.param<double>("/compute_model/batch_size", batch_size, 10000);
    n.param<double>("/compute_model/t_op", t_op, 0.01);
    n.param<double>("/compute_model/app_flo", app_flo, 10000);
    n.param<float>("/compute_model/MIN_POINTS_DIST", MIN_POINTS_DIST, 1);
    n.param<float>("/compute_model/SAFE_DISTANCE", SAFE_DISTANCE, 6);

    ros::Publisher status_pub = n.advertise<computation_msgs::status>("/computation/status", 1000);
    ros::Publisher cooperation_status_pub = n.advertise<cooperative_msgs::status>("/computation/cooperation_status", 1000);
    updated_lane_pub = n.advertise<autoware_msgs::LaneArray>("/computation/updated_lane_waypoints_array", 1000);
    ros::Subscriber computation_sub = n.subscribe("/rx_com", 1000, rxCallback);
    ros::Subscriber ego_path_sub = n.subscribe("/lane_waypoints_array", 1000, egoPathCallback);

    ros::Rate loop_rate(1000);
    LOOP_REPS = app_flo*batch_size;

    cooperation_status.header.stamp = ros::Time::now();
    cooperation_status.LAST_DECISION = cooperative_msgs::status::IDLE;

    while (ros::ok())
    {
        ros::spinOnce();

        cooperation_status_pub.publish(cooperation_status);


        cout.setf(ios_base::fixed); // shows decimals in the output
	    // cout << "loop_reps: " << LOOP_REPS << endl;
    
	    // reference loop
	    clock_t rl_start = clock();
	    // loop index is volatile so that the empty loop isn't optimized away
	    for(volatile uint32_t rl_index = 0; rl_index < LOOP_REPS; ++rl_index) {
	    	// empty loop - just to calculate how much time an empty loop needs
	    }
	    clock_t rl_end = clock();
	    double rl_time = difftime(rl_end, rl_start) / CLOCKS_PER_SEC;
    
	    // output the time the reference loop took
	    // cout << "cl_time:   " << rl_time << endl;
    
	    // flops loop
	    volatile float a = 1.5;
	    volatile float b = 1.6;
	    clock_t fl_start = clock();
	    for(volatile uint32_t fl_index = 0; fl_index < LOOP_REPS; ++fl_index) {
	    	a *= b; // multiplication operation
	    	b += a; // addition operation
	    }
	    clock_t fl_end = clock();
	    double fl_time = difftime(fl_end, fl_start) / CLOCKS_PER_SEC;
	    unsigned long flops = LOOP_REPS / ((fl_time - rl_time) / 2);
    
	    // cout << "fl_time:   " << fl_time << endl;
	    // cout << "flops:     " << flops << endl;


        // TODO
        // ros::Time t0, t1;
        // t0 = ros::Time::now();
        // // std::cout << "------------------T0: " << t0 << std::endl;
        // //ROS_INFO("########################START");
        // for (int i = 0; i <= app_flo*batch_size; i++){
        //     float res = 1.1*1.001; //1 floating point operation
        //     // std::cout << "------------------i: " << i << std::endl;

        // }
        // //ROS_INFO("########################Finish");
        // t1 = ros::Time::now();
        // std::cout << "------------------T1: " << t1 << std::endl;
        computation_msgs::status comp_status;
        comp_status.header.seq = seq;
        // comp_status.header.stamp = t1;
        comp_status.batch_size = batch_size;
        comp_status.t_op = t_op;
        comp_status.app_flo = app_flo;
        // comp_status.execution_time = (t1-t0).toNSec()/1000000000;
        comp_status.execution_time = fl_time;
        comp_status.required_flops = app_flo*batch_size*t_op;
        comp_status.machine_availabel_flops = flops;
        //ros::Duration d(delay_sec);
        status_pub.publish(comp_status);
        updated_lane_pub.publish(updated_lane_waypoints);
        
        seq = seq + 1;
        //if (ros::Time::now() >= vehicle_output_rx_timeStamp + d && vehicle_output_tx_ready)

        //loop_rate.sleep();
    }
    return 0;
}