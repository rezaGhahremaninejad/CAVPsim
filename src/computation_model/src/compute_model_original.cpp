#include "ros/ros.h"
#include "computation_msgs/status.h"
#include "rosgraph_msgs/Clock.h"
#include <ctime>
#include <cmath>

double batch_size, t_op, app_flo;
rosgraph_msgs::Clock _clock;
int seq;
// #define LOOP_REPS 4294967295
int LOOP_REPS;
using namespace std;

void clockCallback(const rosgraph_msgs::Clock::ConstPtr &msg)
{
    _clock = *msg;
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "compute_model");
    ros::NodeHandle n;
    n.param<double>("/compute_model/batch_size", batch_size, 10000);
    n.param<double>("/compute_model/t_op", t_op, 0.01);
    n.param<double>("/compute_model/app_flo", app_flo, 10000);

    ros::Publisher status_pub = n.advertise<computation_msgs::status>("/computation/status", 1000);
    // ros::Subscriber computation_sub = n.subscribe("clock", 1000, clockCallback);

    ros::Rate loop_rate(1000);
    LOOP_REPS = app_flo*batch_size;

    while (ros::ok())
    {
        ros::spinOnce();

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
        seq = seq + 1;
        //if (ros::Time::now() >= vehicle_output_rx_timeStamp + d && vehicle_output_tx_ready)

        //loop_rate.sleep();
    }
    return 0;
}