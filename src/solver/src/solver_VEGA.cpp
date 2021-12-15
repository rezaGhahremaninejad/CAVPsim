#include "ros/ros.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include "std_msgs/String.h"
#include "computation_msgs/status.h"
#include "autoware_msgs/LaneArray.h"
#include "cooperative_msgs/status.h"
#include "communication_msgs/ComMessage.h"
#include "rosgraph_msgs/Clock.h"

cooperative_msgs::status _coop_status;
computation_msgs::status _vega_status;
autoware_msgs::LaneArray _ego_path;
ros::Publisher status_pub, cooperation_status_pub, updated_lane_pub;
ros::Subscriber computation_sub, ego_path_sub;

int FLOP_CALC_PERIOD_SEC;
float t_min;    // Minimum time in msecond required a participant be available to contribute.
float LAST_FLOP_CALC_TIME;
int LOOP_REPS = 1000;
int N_p = 100;
int APPCO = 100;

float c_vp_a, c_vp_b, c_ap_a, c_ap_b;

using namespace std;

struct SOLUTION
{
    autoware_msgs::LaneArray _vehicle_A;
    autoware_msgs::LaneArray _vehicle_B;
};

SOLUTION _solution;

struct POPULATION
{
    std::vector<SOLUTION> _solution;
};

// Mating pool:
POPULATION P_i;

void removeExtraSolutions(const float _size) {
    for (int i = 0; i < P_i._solution.size() - _size; i++) {
        P_i._solution.pop_back();
    }

}

void propSelection(std::vector<float> _fitness, POPULATION _pop) {
    int _min_fitness_idx;
    float _tmp_min_fitness;
    float _total_fitness = 0;
    int TOTAL_IT = _pop._solution.size();
    for (int it = 0; it < TOTAL_IT; it++) {
        for (int i = 0; i < _fitness.size(); i++) {
            if (i == 0) {_tmp_min_fitness = _fitness[0];}
            else {
                if (_fitness[i] < _tmp_min_fitness) {
                    _min_fitness_idx = i;
                    _tmp_min_fitness = _fitness[i];
                }
            }
            
            if (it == 0){_total_fitness = _total_fitness + _fitness[i];}
        }

        _fitness.erase(_fitness.begin() + _min_fitness_idx);
        for (int k = 0; k < round(_total_fitness/(_pop._solution.size()*_tmp_min_fitness)); k++) {
            P_i._solution.push_back(_pop._solution[_min_fitness_idx]);
        }

        if (P_i._solution.size() > _pop._solution.size()) {
            removeExtraSolutions(_pop._solution.size());
            break;
        } else if (P_i._solution.size() == _pop._solution.size()) {
            break;
        }

        TOTAL_IT = _pop._solution.size() - P_i._solution.size();
    }
    // Sorting based on fitness values:
    // for (auto _val : _fitness) {

    // }
    // P_i._solution.push_back()
}

void rxCallback(const communication_msgs::ComMessage::ConstPtr &msg)
{
    // Finding other possible participant in distributed cooperative decision making alg.
    if (msg->msg_source != "") {
        _solution._vehicle_B = msg->ego_path;
        // Find out if this is the leader vehicle for distributed processing.
        if (msg->computation_status.CAV_flop < _vega_status.CAV_flop
        && !msg->computation_status.IS_LEADER) {
            _vega_status.IS_LEADER = true;
            _vega_status.CAV_total_flop = _vega_status.CAV_flop + msg->computation_status.CAV_flop;
        } else if (msg->computation_status.IS_LEADER) {
            // TODO
        }
    }
}

unsigned long calcThisCAV_flp() {
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
    return flops;
}

int calcThisCAV_t_available() {
    return 5000;
}

void egoPathCallback(const autoware_msgs::LaneArray::ConstPtr &msg) {
    _solution._vehicle_A = *msg;   
    // _ego_path.lanes[0].waypoints[j].twist.twist.linear.x 
}

float colCost(const SOLUTION _solution) {
    float _result;
    float d_min = 1.0;
    float d_comMax = 100.0;
    int _idx_A, _idx_B;
    for (int idx = 0; idx < _solution._vehicle_A.lanes[0].waypoints.size() 
    || idx < _solution._vehicle_B.lanes[0].waypoints.size(); idx++) {
        if (idx < _solution._vehicle_A.lanes[0].waypoints.size() - 1) 
        {_idx_A = idx;}
        if (idx < _solution._vehicle_B.lanes[0].waypoints.size() - 1) 
        {_idx_B = idx;}
        float d = pow((pow(_solution._vehicle_B.lanes[0].waypoints[_idx_B].pose.pose.position.x 
        - _solution._vehicle_A.lanes[0].waypoints[_idx_A].pose.pose.position.x, 2)
        + pow(_solution._vehicle_B.lanes[0].waypoints[_idx_B].pose.pose.position.y 
        - _solution._vehicle_A.lanes[0].waypoints[_idx_A].pose.pose.position.y, 2)), 0.5);
        _result = _result + pow(d,2) - (d_min + d_comMax)*d + (d_comMax*d_min);
    }

    return 0;
}

float powCost(const SOLUTION _solution) {
    float _va_speed_sum, _vb_speed_sum;
    float _va_accel_sum, _vb_accel_sum;

    for (auto _val : _solution._vehicle_A.lanes[0].waypoints) {
        _va_speed_sum = _va_speed_sum + _val.twist.twist.linear.x;
    }

    for (auto _val : _solution._vehicle_B.lanes[0].waypoints) {
        _vb_speed_sum = _vb_speed_sum + _val.twist.twist.linear.x;
    }

    // TODO for acceleration
    return c_vp_a * _va_speed_sum + c_vp_b * _vb_speed_sum;
}

float RAND(const float a, const float b) {return a*(rand() % 100 + 1) + b;}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "solver_VEGA");
    ros::NodeHandle n;

    n.param<int>("/compute_model/FLOP_CALC_PERIOD_SEC", FLOP_CALC_PERIOD_SEC, 2);
    n.param<int>("/compute_model/LOOP_REPS", LOOP_REPS, 100);
    n.param<int>("/compute_model/N_p", N_p, 100);
    n.param<int>("/compute_model/APPCO", APPCO, 10);
    n.param<float>("/compute_model/c_vp_a", c_vp_a, 10);
    n.param<float>("/compute_model/c_vp_b", c_vp_b, 10);
    n.param<float>("/compute_model/c_ap_a", c_ap_a, 10);
    n.param<float>("/compute_model/c_ap_b", c_ap_b, 10);

    status_pub = n.advertise<computation_msgs::status>("/computation/status", 1000);
    cooperation_status_pub = n.advertise<cooperative_msgs::status>("/computation/cooperation_status", 1000);
    updated_lane_pub = n.advertise<autoware_msgs::LaneArray>("/computation/updated_lane_waypoints_array", 1000);
    computation_sub = n.subscribe("/rx_com", 1000, rxCallback);
    ego_path_sub = n.subscribe("/lane_waypoints_array", 1000, egoPathCallback);
    _vega_status.id = 1;
    _vega_status.IS_LEADER = false;
    _vega_status.CAV_flop = calcThisCAV_flp();
    _vega_status.CAV_t_available = calcThisCAV_t_available();
    _vega_status.CAV_total_flop = _vega_status.CAV_flop;
    LAST_FLOP_CALC_TIME = ros::Time::now().toSec();

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        if (LAST_FLOP_CALC_TIME + FLOP_CALC_PERIOD_SEC < ros::Time::now().toSec()) {
            std::cout << "------HERE" << std::endl;
            _vega_status.CAV_flop = calcThisCAV_flp();
            _vega_status.CAV_t_available = calcThisCAV_t_available();
        }

        // RAD-VEGA Step 1:
        POPULATION _population;
        for (int i = 0; i < (N_p*_vega_status.CAV_flop*_vega_status.CAV_t_available) / (APPCO*_vega_status.CAV_total_flop*t_min); i++) {
            SOLUTION _tmp_sol;
            autoware_msgs::Waypoint _tmp_waypoint_A = _solution._vehicle_A.lanes[0].waypoints[i];
            autoware_msgs::Waypoint _tmp_waypoint_B = _solution._vehicle_B.lanes[0].waypoints[i];
            // _ego_path.lanes[0].waypoints[j].twist.twist.linear.x 
            for (auto wayPoint : _solution._vehicle_A.lanes[0].waypoints) {
                wayPoint.twist.twist.linear.x = wayPoint.twist.twist.linear.x + RAND(0.01, -0.5);
                _tmp_sol._vehicle_A.lanes[0].waypoints.push_back(wayPoint);
            }

            for (auto wayPoint : _solution._vehicle_B.lanes[0].waypoints) {
                wayPoint.twist.twist.linear.x = wayPoint.twist.twist.linear.x + RAND(0.01, -0.5);
                _tmp_sol._vehicle_B.lanes[0].waypoints.push_back(wayPoint);
            }

            _population._solution.push_back(_tmp_sol);
        }

        // RAD-VEGA Step 2:
        // VEGA STEP 1: M = 3 (We are considering three objectives:
        // reduction in both collision avoidance risk and power consumption and navigation desire)
        int q = _population._solution.size() / 2;
        std::vector<float> _fitness;
        for (int j = 0; j < _population._solution.size(); j++) {
            if (j < q) {_fitness.push_back(colCost(_population._solution[j]));}
            else if (j < q*2) {_fitness.push_back(powCost(_population._solution[j]));}
        }

        // Populating P_i
        propSelection(_fitness, _population);
        status_pub.publish(_vega_status);
        cooperation_status_pub.publish(_coop_status);
        ros::spinOnce();
        //ros::spin();
        loop_rate.sleep();
    }
    return 0;
}