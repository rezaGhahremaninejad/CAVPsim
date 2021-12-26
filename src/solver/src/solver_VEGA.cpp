#include "ros/ros.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include "std_msgs/String.h"
#include "computation_msgs/status.h"
#include "computation_msgs/VEGA_stat.h"
#include "computation_msgs/RAD_VEGA_POPULATION.h"
#include "computation_msgs/RAD_VEGA_SOLUTION.h"
#include "autoware_msgs/LaneArray.h"
#include "cooperative_msgs/status.h"
#include "communication_msgs/ComMessage.h"
#include "rosgraph_msgs/Clock.h"

cooperative_msgs::status _coop_status;
computation_msgs::status _vega_status;
computation_msgs::RAD_VEGA_POPULATION P_i, P_vehicleB;
computation_msgs::RAD_VEGA_SOLUTION _solution;
computation_msgs::VEGA_stat _vega_stat;
autoware_msgs::LaneArray _ego_path;
ros::Publisher status_pub, cooperation_status_pub, updated_lane_pub, vega_stat_pub;
ros::Subscriber computation_sub, ego_path_sub;

int FLOP_CALC_PERIOD_SEC;
int t_min;                        // Minimum time in msecond required a participant be available to contribute.
float LAST_FLOP_CALC_TIME;
int LOOP_REPS = 1000;
int N_p = 100;
int APPCO = 100;

float c_vp_a, c_vp_b, c_ap_a, c_ap_b;

using namespace std;

void removeExtraSolutions(const float _size) {
    for (int i = 0; i < P_i.solution.size() - _size; i++) {
        P_i.solution.pop_back();
    }

}

void propSelection(std::vector<float> _fitness, computation_msgs::RAD_VEGA_POPULATION _pop) {
    
    float _total_fitness = 0;
    int TOTAL_IT = _pop.solution.size();
    std::cout << "------ propSelection start. TOTAL_IT: " << TOTAL_IT << std::endl;
    // std::cout << "------ _fitness.size(): " << _fitness.size() << std::endl;
    float _tmp_min_fitness = 9999999;
    int _min_fitness_idx = 0;
    for (int it = 0; it < TOTAL_IT; it++) {
        _min_fitness_idx = 0;
        _tmp_min_fitness = 9999999;
        for (int i = 0; i < _fitness.size(); i++) {
            std::cout << "------ _fitness[i]: " << _fitness[i] << " --- i: " << i << std::endl;
            
            if (_fitness[i] < _tmp_min_fitness) {
                _min_fitness_idx = i;
                _tmp_min_fitness = _fitness[i];
            }

            if (it == 0){_total_fitness = _total_fitness + _fitness[i];}
        }

        std::cout << "------ propSelection 2. it: " << it << std::endl;
        // std::cout << "------ _min_fitness_idx: " << _min_fitness_idx << std::endl;

        
        if (_min_fitness_idx >= 0) {_fitness.erase(_fitness.begin() + _min_fitness_idx);}
        std::cout << "------ propSelection 3. it: " << it << std::endl;

        std::cout << "------ _total_fitness: " << _total_fitness << std::endl;
        std::cout << "------ _tmp_min_fitness: " << _tmp_min_fitness << std::endl;
        std::cout << "------ kMAX: " << round(_total_fitness/(_pop.solution.size()*_tmp_min_fitness)) << std::endl;
        for (int k = 0; k < round(_total_fitness/(_pop.solution.size()*_tmp_min_fitness)); k++) {
            P_i.solution.push_back(_pop.solution[_min_fitness_idx]);
        }
        std::cout << "------ propSelection 4. it: " << it << std::endl;

        if (P_i.solution.size() > _pop.solution.size()) {
            removeExtraSolutions(_pop.solution.size());
            break;
        } else if (P_i.solution.size() == _pop.solution.size()) {
            break;
        }

        TOTAL_IT = _pop.solution.size() - P_i.solution.size();
        std::cout << "------ TOTAL_IT: " << TOTAL_IT << std::endl;
        // std::cout << "------ P_i.solution.size(): " << P_i.solution.size() << std::endl;
        // std::cout << "------ _fitness.size(): " << _fitness.size() << std::endl;
    }
    // std::cout << "------ propSelection end" << std::endl;
}

void rxCallback(const communication_msgs::ComMessage::ConstPtr &msg)
{
    // Finding other possible participant in distributed cooperative decision making alg.
    P_vehicleB = msg->computation_status.P; 
    _solution.vehicleB = msg->ego_path;
    // Find out if this is the leader vehicle for distributed processing.
    if (msg->computation_status.CAV_flop < _vega_status.CAV_flop
    && !msg->computation_status.IS_LEADER 
    && _vega_status.CAV_total_flop != 0) {
        _vega_status.IS_LEADER = true;
        _vega_status.CAV_total_flop = _vega_status.CAV_flop + msg->computation_status.CAV_flop;
    } else if (msg->computation_status.IS_LEADER) {
        // TODO
        _vega_status.CAV_total_flop = _vega_status.CAV_flop + msg->computation_status.CAV_flop;
        _vega_status.IS_LEADER = false;
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

unsigned long calcThisCAV_t_available() {
    unsigned long _res = 5000;
    return _res;
}

void egoPathCallback(const autoware_msgs::LaneArray::ConstPtr &msg) {
    _solution.vehicleA = *msg;   
    // _ego_path.lanes[0].waypoints[j].twist.twist.linear.x 
}

float colCost(const computation_msgs::RAD_VEGA_SOLUTION _sol) {
    float _result;
    float d_min = 1.0;
    float d_comMax = 100.0;
    std::vector<float> _exe_time_vA, _exe_time_vB;
    int _idx_A, _idx_B;
    for (int idx = 0; idx < _sol.vehicleA.lanes[0].waypoints.size() 
    || idx < _sol.vehicleB.lanes[0].waypoints.size(); idx++) {
        // std::cout << "-------colcost  idx: " <<  idx << std::endl;
        if (idx < _sol.vehicleA.lanes[0].waypoints.size() - 1) 
        {_idx_A = idx;}
        if (idx < _sol.vehicleB.lanes[0].waypoints.size() - 1) 
        {_idx_B = idx;}
        float dA = pow((pow(_sol.vehicleB.lanes[0].waypoints[_idx_A].pose.pose.position.x 
        - _sol.vehicleA.lanes[0].waypoints[_idx_A - 1].pose.pose.position.x, 2)
        + pow(_sol.vehicleB.lanes[0].waypoints[_idx_A].pose.pose.position.y 
        - _sol.vehicleA.lanes[0].waypoints[_idx_A - 1].pose.pose.position.y, 2)), 0.5);
        float _tA = (pow(_sol.vehicleA.lanes[0].waypoints[_idx_A].twist.twist.linear.x, 2) - 
        pow(_sol.vehicleA.lanes[0].waypoints[_idx_A - 1].twist.twist.linear.x, 2)) / (2*dA);
        _exe_time_vA.push_back(_tA);
        float dB = pow((pow(_sol.vehicleB.lanes[0].waypoints[_idx_B].pose.pose.position.x 
        - _sol.vehicleA.lanes[0].waypoints[_idx_B - 1].pose.pose.position.x, 2)
        + pow(_sol.vehicleB.lanes[0].waypoints[_idx_B].pose.pose.position.y 
        - _sol.vehicleA.lanes[0].waypoints[_idx_B - 1].pose.pose.position.y, 2)), 0.5);
        float _tB = (pow(_sol.vehicleA.lanes[0].waypoints[_idx_B].twist.twist.linear.x, 2) - 
        pow(_sol.vehicleB.lanes[0].waypoints[_idx_B - 1].twist.twist.linear.x, 2)) / (2*dB);
        _exe_time_vB.push_back(_tB);
    }

    _idx_A = 0;
    _idx_B = 0;
    float _t_var = 0.1;
    for (int idx = 0; idx < _sol.vehicleA.lanes[0].waypoints.size() 
    || idx < _sol.vehicleB.lanes[0].waypoints.size(); idx++) {
        if (idx < _sol.vehicleA.lanes[0].waypoints.size() - 1) 
        {_idx_A = idx;}
        if (idx < _sol.vehicleB.lanes[0].waypoints.size() - 1) 
        {_idx_B = idx;}
        if (abs(_exe_time_vA[_idx_A] - abs(_exe_time_vB[_idx_B]) < _t_var)) {
            _result = _result + pow(pow(_sol.vehicleA.lanes[0].waypoints[_idx_A].pose.pose.position.x - _sol.vehicleB.lanes[0].waypoints[_idx_B].pose.pose.position.x, 2)
            + pow(_sol.vehicleA.lanes[0].waypoints[_idx_A].pose.pose.position.y - _sol.vehicleB.lanes[0].waypoints[_idx_B].pose.pose.position.y, 2), 0.5);
        } else if (_idx_B > 1 && abs(_exe_time_vA[_idx_A] - abs(_exe_time_vB[_idx_B - 1]) < _t_var)) {
            _result = _result + pow(pow(_sol.vehicleA.lanes[0].waypoints[_idx_A].pose.pose.position.x - _sol.vehicleB.lanes[0].waypoints[_idx_B - 1].pose.pose.position.x, 2)
            + pow(_sol.vehicleA.lanes[0].waypoints[_idx_A].pose.pose.position.y - _sol.vehicleB.lanes[0].waypoints[_idx_B - 1].pose.pose.position.y, 2), 0.5);
        } else if (_idx_B < _exe_time_vB.size() && abs(_exe_time_vA[_idx_A] - abs(_exe_time_vB[_idx_B - 1]) < _t_var)) {
            _result = _result + pow(pow(_sol.vehicleA.lanes[0].waypoints[_idx_A].pose.pose.position.x - _sol.vehicleB.lanes[0].waypoints[_idx_B + 1].pose.pose.position.x, 2)
            + pow(_sol.vehicleA.lanes[0].waypoints[_idx_A].pose.pose.position.y - _sol.vehicleB.lanes[0].waypoints[_idx_B + 1].pose.pose.position.y, 2), 0.5);
        }
    }

    return _result;
}

float powCost(const computation_msgs::RAD_VEGA_SOLUTION _sol) {
    float _va_speed_sum = 0;
    float _vb_speed_sum = 0;
    float _va_accel_sum = 0;
    float _vb_accel_sum = 0;

    for (auto _val : _sol.vehicleA.lanes[0].waypoints) {
        _va_speed_sum = _va_speed_sum + c_vp_a*_val.twist.twist.linear.x;
        // std::cout << "--------_val.twist.twist.linear.x: " << _val.twist.twist.linear.x << std::endl;
        // std::cout << "--------_va_speed_sum: " << _va_speed_sum << std::endl;
    }

    for (auto _val : _sol.vehicleB.lanes[0].waypoints) {
        _vb_speed_sum = _vb_speed_sum + c_vp_b*_val.twist.twist.linear.x;
        // std::cout << "--------_vb_speed_sum: " << _vb_speed_sum << std::endl;
    }

    // TODO for acceleration
    float _res = _va_speed_sum + _vb_speed_sum;
    // std::cout << "--------_va_speed_sum: " << _va_speed_sum << std::endl;
    // std::cout << "--------_vb_speed_sum: " << _vb_speed_sum << std::endl;
    // std::cout << "--------c_vp_a: " << c_vp_a << std::endl;
    // std::cout << "--------c_vp_b: " << c_vp_b << std::endl;
    // std::cout << "--------_res: " << _res << std::endl;
    return _res;
}

int RAND(const int a, const int b) 
{return b + ( std::rand() % ( a - b + 1));}
// {return (rand() % a + b);}

computation_msgs::RAD_VEGA_POPULATION corssAndMutation(computation_msgs::RAD_VEGA_POPULATION _inpop) {
    computation_msgs::RAD_VEGA_POPULATION _outpop;
    int idx = 0;
    // std::cout << "------ _inpop.solution.size(): " << _inpop.solution.size() << std::endl;
    while (idx < _inpop.solution.size()) {
        computation_msgs::RAD_VEGA_SOLUTION _tmp_solution;
        int crossCandidIdx_1 = RAND(_inpop.solution.size() - 1, 0);
        int crossCandidIdx_2 = RAND(_inpop.solution.size() - 1, 0);
        // std::cout << "------ cross 2 _inpop.solution.size(): " << _inpop.solution.size() << std::endl;
        // std::cout << "------ cross 2 crossCandidIdx_1: " << crossCandidIdx_1 << std::endl;
        // std::cout << "------ cross 2 crossCandidIdx_2: " << crossCandidIdx_2 << std::endl;
        _tmp_solution.vehicleA = _inpop.solution[crossCandidIdx_1].vehicleA;
        _tmp_solution.vehicleB = _inpop.solution[crossCandidIdx_2].vehicleB;
        // std::cout << "------ cross 3 idx: " << idx << std::endl;
        _outpop.solution.push_back(_tmp_solution);
        // std::cout << "------ cross 4 idx" << idx << std::endl;
        idx++;
    }
    return _outpop;
}

computation_msgs::RAD_VEGA_SOLUTION findBestSolution(const computation_msgs::RAD_VEGA_POPULATION _pop) {
    computation_msgs::RAD_VEGA_SOLUTION _sol;
    computation_msgs::RAD_VEGA_POPULATION _pareto_frontier_set;
    // int candIdx = 0;
    float MIN_COL_COST = 99999999;
    float MIN_POW_COST = 99999999;
    // Finding pareto frontier:
    for (int i = 0; i < _pop.solution.size(); i++) {
        // candIdx = RAND(_pop.solution.size(), 0);
        float _col_cost = colCost(_pop.solution[i]); 
        float _pow_cost = powCost(_pop.solution[i]); 
        // std::cout << "-----_col_cost: " << _col_cost << std::endl;
        // std::cout << "-----_pow_cost: " << _pow_cost << std::endl;
        if (_col_cost < MIN_COL_COST) {
            if (_pow_cost < MIN_POW_COST) {
                MIN_COL_COST = _col_cost;
                MIN_POW_COST = _pow_cost;
                _pareto_frontier_set.solution.clear();
            } else {
                _pareto_frontier_set.solution.push_back(_pop.solution[i]);
            }
        } else if (_pow_cost < MIN_POW_COST) {
            _pareto_frontier_set.solution.push_back(_pop.solution[i]);
        }
    }

    if (_pareto_frontier_set.solution.size() > 1) {
        int idx = RAND(_pareto_frontier_set.solution.size() - 1, 0);
        // std::cout << "-----_pop.solution.size(): " << _pop.solution.size() << std::endl;
        // std::cout << "-----_pareto_frontier_set.solution.size(): " << _pareto_frontier_set.solution.size() << std::endl;
        // std::cout << "-----idx: " << idx << std::endl;
        _sol = _pareto_frontier_set.solution[idx];
    }
    return _sol;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "solver_VEGA");
    ros::NodeHandle n;

    n.param<int>("/compute_model/FLOP_CALC_PERIOD_SEC", FLOP_CALC_PERIOD_SEC, 2);
    n.param<int>("/compute_model/LOOP_REPS", LOOP_REPS, 10000000);
    n.param<int>("/compute_model/N_p", N_p, 100);
    n.param<int>("/compute_model/APPCO", APPCO, 10);
    n.param<float>("/compute_model/c_vp_a", c_vp_a, 0.1);
    n.param<float>("/compute_model/c_vp_b", c_vp_b, 0.1);
    n.param<float>("/compute_model/c_ap_a", c_ap_a, 0.1);
    n.param<float>("/compute_model/c_ap_b", c_ap_b, 0.1);
    n.param<int>("/compute_model/t_min", t_min, 1000);

    status_pub = n.advertise<computation_msgs::status>("/computation/status", 1000);
    vega_stat_pub = n.advertise<computation_msgs::VEGA_stat>("/computation/VEGA_stat", 1000);
    cooperation_status_pub = n.advertise<cooperative_msgs::status>("/computation/cooperation_status", 1000);
    updated_lane_pub = n.advertise<autoware_msgs::LaneArray>("/computation/updated_lane_waypoints_array", 1000);
    computation_sub = n.subscribe("/rx_com", 1000, rxCallback);
    ego_path_sub = n.subscribe("/lane_waypoints_array", 1000, egoPathCallback);
    _vega_status.id = 1;
    _vega_status.IS_LEADER = false;
    _vega_status.CAV_total_flop = calcThisCAV_flp();
    srand((unsigned) time(0));
    ros::Rate loop_rate(10);
    // std::cout << "--------------RAND(100, 0): " << RAND(100, 0) << std::endl;
    while (ros::ok())
    {
        _vega_status.CAV_flop = calcThisCAV_flp();
        // std::cout << "------: _vega_status.CAV_flop: " << _vega_status.CAV_flop << std::endl;
        _vega_status.CAV_t_available = calcThisCAV_t_available();
        // std::cout << "------: _vega_status.CAV_t_available: " << _vega_status.CAV_t_available << std::endl;
        // std::cout << "------: _1: " << N_p*_vega_status.CAV_flop*_vega_status.CAV_t_available << std::endl;
        // std::cout << "------: _2: " << APPCO*_vega_status.CAV_total_flop*t_min << std::endl;
        _vega_stat.N_i = (N_p*_vega_status.CAV_flop*_vega_status.CAV_t_available) / (APPCO*_vega_status.CAV_total_flop*t_min);
        // std::cout << "------: _1/2: " << _vega_stat.N_i << std::endl;
        // _vega_status.CAV_t_available = 200;
        // }

        // RAD-VEGA Step 1:
        std::cout << "------ RAD-VEGA Step 1. " << std::endl;
       computation_msgs::RAD_VEGA_POPULATION _population;
        for (int i = 0; i < (N_p*_vega_status.CAV_flop*_vega_status.CAV_t_available) / (APPCO*_vega_status.CAV_total_flop*t_min); i++) {
            computation_msgs::RAD_VEGA_SOLUTION _tmp_sol;
            if (_solution.vehicleA.lanes.size() != 0 && _solution.vehicleB.lanes.size() != 0) {
                autoware_msgs::Lane _tmp_lane;
                _tmp_sol.vehicleA.lanes.push_back(_tmp_lane);
                _tmp_sol.vehicleB.lanes.push_back(_tmp_lane);
                for (auto wayPoint : _solution.vehicleA.lanes[0].waypoints) {
                    wayPoint.twist.twist.linear.x = wayPoint.twist.twist.linear.x + 0.1*(RAND(10, 0) - 5);
                    _tmp_sol.vehicleA.lanes[0].waypoints.push_back(wayPoint);
                }

                for (auto wayPoint : _solution.vehicleB.lanes[0].waypoints) {
                    wayPoint.twist.twist.linear.x = wayPoint.twist.twist.linear.x + 0.1*(RAND(10, 0) - 5);
                    _tmp_sol.vehicleB.lanes[0].waypoints.push_back(wayPoint);
                }

                _population.solution.push_back(_tmp_sol);
            }
        }

        // RAD-VEGA Step 2:
        // VEGA STEP 1: M = 2 (We are considering two objectives:
        // reduction in both collision avoidance risk and power consumption and navigation desire)
        std::cout << "------ RAD-VEGA Step 2. " << std::endl;
        int q = _population.solution.size() / 2;
        std::vector<float> _fitness;
        for (int j = 0; j < _population.solution.size(); j++) {
            // std::cout << "------ RAD-VEGA Step 2. j: " << j << std::endl;
            // std::cout << "------ _population.solution[j]: " << _population.solution[j].vehicleA.lanes[0].waypoints[0].twist.twist.linear.x << std::endl;
            if (j < q) {
                _fitness.push_back(colCost(_population.solution[j]));
            } else if (j < _population.solution.size()) {
                _fitness.push_back(powCost(_population.solution[j]));
            }
        }

        // Populating P_i
        std::cout << "------ RAD-VEGA Step 3.1. " << std::endl;
        propSelection(_fitness, _population);
        std::cout << "------ RAD-VEGA Step 3.2. " << std::endl;
        cooperation_status_pub.publish(_coop_status);
        
        _vega_stat.partner_solution_population_size = P_vehicleB.solution.size();
        _vega_stat.this_solution_population_size = _population.solution.size();
        _vega_stat.total_solution_population_size = _population.solution.size();
        
        // RAD-VEGA Step 3:
        _vega_status.P.solution.clear();
        _vega_status.P.solution.insert(_vega_status.P.solution.begin(), _population.solution.begin(), _population.solution.end());
        if (_vega_status.IS_LEADER && P_vehicleB.solution.size() > 0) {
            computation_msgs::RAD_VEGA_POPULATION P;
            P = _population;
            // std::cout << "------ AS parent. 1, P_vehicleB.solution.size()" << P_vehicleB.solution.size() << std::endl;
            // std::cout << "------ AS parent. 1.1, P.solution.size()" << P.solution.size() << std::endl;
            P.solution.insert(P.solution.end(), P_vehicleB.solution.begin(), P_vehicleB.solution.end());
            P_vehicleB.solution.clear();
            P = corssAndMutation(P);
            _vega_stat.total_solution_population_size = P_vehicleB.solution.size();
            // std::cout << "------ AS parent. 2 P.solution.size()" << P.solution.size() << std::endl;
            // std::cout << "------ AS parent. 2.1 VA Speed test" << P.solution[0].vehicleA.lanes[0].waypoints[0].twist.twist.linear.x << std::endl;
            // std::cout << "------ AS parent. 2.1 VB Speed test" << P.solution[0].vehicleB.lanes[0].waypoints[0].twist.twist.linear.x << std::endl;
            // std::cout << "------ AS parent. 2.1 colCost" << colCost(P.solution[0]) << std::endl;
            // std::cout << "------ AS parent. 2.1 powCost" << powCost(P.solution[0]) << std::endl;
            // std::cout << "------ AS parent. 3.1, _sol.vehicleA.lanes.size(): " << _sol.vehicleA.lanes.size() << std::endl;
            // std::cout << "------ AS parent. 3.1, _sol.vehicleB.lanes.size(): " << _sol.vehicleB.lanes.size() << std::endl;
            computation_msgs::RAD_VEGA_SOLUTION _sol = findBestSolution(P);
            if (_sol.vehicleA.lanes.size() > 0 
            && _sol.vehicleB.lanes.size() > 0) {
                std::cout << "------ AS parent. 3" << std::endl;
                _vega_stat.corrected_path_vehicleA = _sol.vehicleA;
                _vega_stat.corrected_path_vehicleB = _sol.vehicleB;
                // std::cout << "------ AS parent. 3.15 colCost(_sol): " << colCost(_sol) << std::endl;
                // std::cout << "------ AS parent. 3.15 powCost(_sol): " << powCost(_sol) << std::endl;
                _vega_stat.corrected_path_col_cost = colCost(_sol);
                _vega_stat.corrected_path_pow_cost = powCost(_sol);
                // std::cout << "------ AS parent. 3.2" << std::endl;
                _vega_stat.initial_path_vehicleA = _solution.vehicleA;
                _vega_stat.initial_path_vehicleB = _solution.vehicleB;
                _vega_stat.initial_path_col_cost = colCost(_solution);
                _vega_stat.initial_path_pow_cost = powCost(_solution);
            }
            std::cout << "------ AS parent. 4" << std::endl;
        }

        status_pub.publish(_vega_status);
        vega_stat_pub.publish(_vega_stat);
        ros::spinOnce();
        //ros::spin();
        loop_rate.sleep();
        std::cout << "------ END. " << std::endl;
    }
    return 0;
}