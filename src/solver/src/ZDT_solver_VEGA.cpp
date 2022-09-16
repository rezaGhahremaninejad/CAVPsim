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
#include <iostream>
#include <algorithm>
cooperative_msgs::status _coop_status;
computation_msgs::status _vega_status;
computation_msgs::RAD_VEGA_POPULATION P_i, P_vehicleB;
computation_msgs::RAD_VEGA_SOLUTION _solution;
computation_msgs::VEGA_stat _vega_stat;
autoware_msgs::LaneArray _ego_path;
ros::Publisher status_pub, cooperation_status_pub, updated_lane_pub, vega_stat_pub;
ros::Subscriber computation_sub, ego_path_sub;

int FLOP_CALC_PERIOD_SEC;
int t_min = 1000;                        // Minimum time in msecond required a participant be available to contribute.
float LAST_FLOP_CALC_TIME;
int LOOP_REPS = 1000;
int N_p = 100;
int APPCO = 10;
int MAX_POP_SIZE = 300;
float c_vp_a, c_vp_b, c_ap_a, c_ap_b;

using namespace std;

void removeExtraSolutions(const float _size) {
    for (int i = 0; i < _size; i++) {
        P_i.solution.pop_back();
    }

}

float zdt1f1Objective(const computation_msgs::RAD_VEGA_SOLUTION _sol) {
    // std::cout << "zdt1f1Objective--------" << _sol.vehicleA.lanes.at(0).waypoints.at(0).twist.twist.linear.x << std::endl;
    float _res = 9999999.0;
    if (_sol.vehicleA.lanes.size() != 0) {
        _res = _sol.vehicleA.lanes.at(0).waypoints.at(0).twist.twist.linear.x;
    }
    return _res;
}

float sumWayPointsTwistOnX(computation_msgs::RAD_VEGA_SOLUTION _sol, float start_idx, float end_idx) {
    float _res = 0;
    for (int i = start_idx; i < end_idx; i++) {
        _res = _res + _sol.vehicleA.lanes.at(0).waypoints.at(i).twist.twist.linear.x
        + _sol.vehicleB.lanes.at(0).waypoints.at(i).twist.twist.linear.x;
    }
    // std::cout << "sumWayPointsTwistOnX--------" << _res << std::endl;
    return _res + _sol.vehicleB.lanes.at(0).waypoints.at(0).twist.twist.linear.x;
}

float zdt1f2Objective(const computation_msgs::RAD_VEGA_SOLUTION _sol) {
    float sum = sumWayPointsTwistOnX(_sol
    , 1
    , _sol.vehicleA.lanes.at(0).waypoints.size());
    // sum = sum + sumWayPointsTwistOnX(_sol, 0, _sol.vehicleB.lanes.at(0).waypoints.size());
    float g = 1.0 + (9.0/((_sol.vehicleA.lanes.at(0).waypoints.size() + _sol.vehicleB.lanes.at(0).waypoints.size()) - 1.0))*sum;
    float _res;
    // std::cout << "---------------g: " << g << std::endl;
    _res = g*(1.0 - pow((_sol.vehicleA.lanes.at(0).waypoints.at(0).twist.twist.linear.x/g),0.5));
    if (_res < 1.0) {
        // std::cout << "------------------BINGO: " << _res << std::endl;
    }
    // std::cout << "------------------g: " << g << std::endl;
    return _res;
}

void propSelection(computation_msgs::RAD_VEGA_POPULATION _pop) {
    // We know that the half of the _pop should be evauated by f1
    // And the other half by f2 so:
    std::vector<float> _fitness_f1, _fitness_f2;
    computation_msgs::RAD_VEGA_POPULATION _pop_f1;
    computation_msgs::RAD_VEGA_POPULATION _pop_f2;

    // Global obj to save the new population, P_i:
    P_i.solution.clear();
    // std::cout << "---------------------1_pop.solution.size(): " << _pop.solution.size() << std::endl;


    // initializing tmp vars:
    float _total_fitness = 0;
    float POP_PORP = 0.05;
    float _tmp_min_fitness = 9999999;
    float _last_tmp_min_fitness;
    int _min_fitness_idx = 0;
    for (int i = 0; i < _pop.solution.size(); i++) {
        if (i < _pop.solution.size()/2) {
            _fitness_f1.push_back(zdt1f1Objective(_pop.solution.at(i)));
            _pop_f1.solution.push_back(_pop.solution.at(i));
        } else {
            // std::cout << "----------------zdt1f2Objective(_pop.solution.at(i)): " << zdt1f2Objective(_pop.solution.at(i)) << std::endl;
            _fitness_f2.push_back(zdt1f2Objective(_pop.solution.at(i)));
            _pop_f2.solution.push_back(_pop.solution.at(i));
        }
    }
    
    // std::cout << "---------------------_pop_f1.solution.size(): " << _pop_f1.solution.size() << std::endl;
    // std::cout << "---------------------_pop_f2.solution.skize(): " << _pop_f2.solution.size() << std::endl;

    int IT_COUNT_f1 = _pop_f1.solution.size(); 
    int IT_COUNT_f2 = _pop_f2.solution.size(); 

    int REMAINING_f1 = IT_COUNT_f1; 
    int REMAINING_f2 = IT_COUNT_f2; 

    for (int it = 0; it < IT_COUNT_f1; it++) {
        // std::cout << "---------------------f1_it: " << it << std::endl;
        // std::cout << "---------------------f1_P_i.solution.size(): " << P_i.solution.size() << std::endl;

        _total_fitness = 0; 
        _tmp_min_fitness = 9999999;
        _min_fitness_idx = 0;
        for (int i = 0; i < _fitness_f1.size(); i++) {            
            if (_fitness_f1[i] < _tmp_min_fitness) {
                _min_fitness_idx = i;
                _tmp_min_fitness = _fitness_f1[i];
            }
            _total_fitness = _total_fitness + _fitness_f1[i];
        }

        // std::cout << "---------------------f1_total_fitness: " << _total_fitness << std::endl;
        // std::cout << "---------------------f1_tmp_min_fitness: " << _tmp_min_fitness << std::endl;
        if (_tmp_min_fitness != _last_tmp_min_fitness) {

            int _number_of_copies = 0;
            _number_of_copies = round(POP_PORP*REMAINING_f1);
            // std::cout << "---------------------f1_number_of_copies: " << _number_of_copies << std::endl;
            // if (_tmp_min_fitness == 0 ) { _number_of_copies = round(POP_PORP*REMAINING_f1);} 
            // else {
            //     if (round(_total_fitness/(REMAINING_f1*_tmp_min_fitness)) < round(POP_PORP*REMAINING_f1)) 
            //     {_number_of_copies = round(_total_fitness/(POP_PORP*REMAINING_f1*_tmp_min_fitness));}
            //     else {_number_of_copies = round(POP_PORP*REMAINING_f1);}
            // }

            // if (_number_of_copies == 0) {
            //     P_i.solution.push_back(P_i.solution[0]);
            // }

            for (int k = 0; k < _number_of_copies; k++) {
                P_i.solution.push_back(_pop_f1.solution[_min_fitness_idx]);
            }

            // REMAINING_f1 = MAX_POP_SIZE/2 - P_i.solution.size() - 1;
            REMAINING_f1 = IT_COUNT_f1 - it;
        }
        
        _fitness_f1.erase(_fitness_f1.begin() + _min_fitness_idx);
        _pop_f1.solution.erase(_pop_f1.solution.begin() + _min_fitness_idx);
        _last_tmp_min_fitness = _tmp_min_fitness;

        // if (P_i.solution.size() >= MAX_POP_SIZE || IT_COUNT_f2 <= 0) {
        // if (P_i.solution.size() >= MAX_POP_SIZE/2) {
        //     for (int i = MAX_POP_SIZE/2; i < P_i.solution.size(); i++) {
        //         if (P_i.solution.size() != 2) {
        //             P_i.solution.pop_back();
        //         } else {
        //             // break;
        //         }
        //     }
        //     // break;
        // }
    } 

    while(P_i.solution.size() > MAX_POP_SIZE/2) {
        P_i.solution.pop_back();
    }
    // if (P_i.solution.size() < _pop_f1.solution.size()) {
    //     for (int i = 0; i < _pop_f1.solution.size() - P_i.solution.size(); i++) {
    //         P_i.solution.push_back(P_i.solution[0]);
    //     }
    // } else {
    //     // while (P_i.solution.size() > _pop_f2.solution.size()) {
    //     //     P_i.solution.pop_back();
    //     // }
    // }

    

    // std::cout << "---------------------f1_P_i.solution.size(): " << P_i.solution.size() << std::endl;
    _last_tmp_min_fitness = 0;
    int _current_p_i_size = P_i.solution.size();

    for (int it = 0; it < IT_COUNT_f2; it++) {
        // std::cout << "---------------------f2_it: " << it << std::endl;
        // std::cout << "---------------------f2_P_i.solution.size(): " << P_i.solution.size() << std::endl;
        _total_fitness = 0;
        _tmp_min_fitness = 9999999;
        _min_fitness_idx = 0;
        for (int i = 0; i < _fitness_f2.size(); i++) {            
            if (_fitness_f2[i] < _tmp_min_fitness) {
                _min_fitness_idx = i;
                _tmp_min_fitness = _fitness_f2[i];
            }
            _total_fitness = _total_fitness + _fitness_f2[i];
        }

        // if (it < 2) {
        //     std::cout << "---------------------f2_it: " << it << std::endl;
        //     std::cout << "---------------------f2_total_fitness: " << _total_fitness << std::endl;
        //     std::cout << "---------------------f2_tmp_min_fitness: " << _tmp_min_fitness << std::endl;
        // }

        if (_tmp_min_fitness != _last_tmp_min_fitness) {

            int _number_of_copies = 0;
            _number_of_copies = round(POP_PORP*REMAINING_f2);
            // std::cout << "---------------------f2_number_of_copies: " << _number_of_copies << std::endl;
            // if (_tmp_min_fitness == 0 ) { _number_of_copies = round(POP_PORP*REMAINING_f2);} 
            // else {
            //     if (round(_total_fitness/(REMAINING_f2*_tmp_min_fitness)) < round(POP_PORP*REMAINING_f2)) 
            //     {_number_of_copies = round(_total_fitness/(POP_PORP*REMAINING_f2*_tmp_min_fitness));}
            //     else {_number_of_copies = round(POP_PORP*REMAINING_f2);}
            // }

            // if (_number_of_copies == 0) {
            //     P_i.solution.push_back(P_i.solution[0]);
            // }

            for (int k = 0; k < _number_of_copies; k++) {
                P_i.solution.push_back(_pop_f2.solution[_min_fitness_idx]);
            }

            // std::cout << "---f2__number_of_copies: " << _number_of_copies << std::endl;

            // REMAINING_f2 = MAX_POP_SIZE/2 - P_i.solution.size() + _current_p_i_size - 1;
            REMAINING_f2 = IT_COUNT_f2 - it;

        }
        _fitness_f2.erase(_fitness_f2.begin() + _min_fitness_idx);
        _pop_f2.solution.erase(_pop_f2.solution.begin() + _min_fitness_idx);
        _last_tmp_min_fitness = _tmp_min_fitness;

        // if (P_i.solution.size() >= MAX_POP_SIZE/2 || IT_COUNT_f2 <= 0) {
        // if (P_i.solution.size() >= MAX_POP_SIZE) {
        //     for (int i = MAX_POP_SIZE; i < P_i.solution.size(); i++) {
        //         if (P_i.solution.size() != 1) {
        //             P_i.solution.pop_back();
        //         } else {
        //             // break;
        //         }
        //     }
        // }
    }   

    while(P_i.solution.size() > MAX_POP_SIZE) {
        P_i.solution.pop_back();
    }

    // if (P_i.solution.size() < _pop_f2.solution.size() + _pop_f1.solution.size()) {
    //     for (int i = 0; i < _pop_f1.solution.size() + _pop_f2.solution.size() - P_i.solution.size(); i++) {
    //         P_i.solution.push_back(P_i.solution[0]);
    //     }
    // } else {
    //     // while (P_i.solution.size() > _pop_f1.solution.size() + _pop_f2.solution.size()) {
    //     //     P_i.solution.pop_back();
    //     // }
    // }
    
    // std::cout << "---------------------f2_P_i.solution.size(): " << P_i.solution.size() << std::endl;
    
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
    // std::cout << "Getting init pop" << std::endl;
    ego_path_sub.shutdown();
    // _ego_path.lanes[0].waypoints[j].twist.twist.linear.x 
}

int RAND(const int a, const int b) 
{
    return b + ( std::rand() % ( a - b + 1));
}
// {return (rand() % a + b);}

computation_msgs::RAD_VEGA_POPULATION crossAndMate(computation_msgs::RAD_VEGA_POPULATION _inpop) {
    computation_msgs::RAD_VEGA_POPULATION _outpop;
    int idx = 0;
    // std::cout << "------ _inpop.solution.size(): " << _inpop.solution.size() << std::endl;
    // for (auto _sol : _inpop.solution) {
    //     std::cout << "-----------------------inpop_sol_f1: " << zdt1f1Objective(_sol) << std::endl;
    //     std::cout << "-----------------------inpop_sol_f2: " << zdt1f2Objective(_sol) << std::endl;
    // }
    int crossCandidIdx_1 = 0;
    int crossCandidIdx_2 = 0;
    int mut_count = 0;
    int MAX_MUT_COUNT = 300;
    // std::cout << "----------_inpop.solution.size(): " << _inpop.solution.size() << std::endl;
    // while (mut_count < MAX_MUT_COUNT) {
    // std::cout << "------------1:_inpop.solution.size():" <<  _inpop.solution.size() << std::endl;

    while (idx < MAX_MUT_COUNT) {
        // std::cout << "------------1idx:" <<  idx << std::endl;
        computation_msgs::RAD_VEGA_SOLUTION _tmp_solution;
        NEXT_CANDIDATE:
        // std::cout << "------------2idx:" <<  idx << std::endl;
        crossCandidIdx_1 = round((static_cast <float> (rand()) / static_cast <float> (RAND_MAX))*_inpop.solution.size());
        crossCandidIdx_2 = round((static_cast <float> (rand()) / static_cast <float> (RAND_MAX))*_inpop.solution.size());
        // std::cout << "------------3idx:" <<  idx << std::endl;
        int _so_tmp = _inpop.solution.size() - 1;
        crossCandidIdx_1 = std::min(crossCandidIdx_1,_so_tmp);
        crossCandidIdx_2 = std::min(crossCandidIdx_2,_so_tmp);
        if (zdt1f1Objective(_inpop.solution[crossCandidIdx_1]) != zdt1f1Objective(_inpop.solution[crossCandidIdx_2])
        && zdt1f2Objective(_inpop.solution[crossCandidIdx_1]) != zdt1f2Objective(_inpop.solution[crossCandidIdx_2])) {
            _tmp_solution.vehicleA = _inpop.solution[crossCandidIdx_1].vehicleA;
            _tmp_solution.vehicleB = _inpop.solution[crossCandidIdx_2].vehicleB;
            // _tmp_solution = _inpop.solution[idx];
            int new_childs_try_count = 0;
            TRY_NEW_CHILDS:
            // std::cout << "------------NEW CHILD idx:" <<  idx << std::endl;

            // (new_childs_try_count == 10*_inpop.solution.size())
            bool F1_P1 = false;
            bool F1_P2 = false;
            bool F2_P1 = false;
            bool F2_P2 = false;
            if (zdt1f1Objective(_tmp_solution) < zdt1f1Objective(_inpop.solution[crossCandidIdx_1])) {F1_P1 = true;}
            if (zdt1f1Objective(_tmp_solution) < zdt1f1Objective(_inpop.solution[crossCandidIdx_2])) {F1_P2 = true;}
            if (zdt1f2Objective(_tmp_solution) < zdt1f2Objective(_inpop.solution[crossCandidIdx_1])) {F2_P1 = true;}
            if (zdt1f2Objective(_tmp_solution) < zdt1f2Objective(_inpop.solution[crossCandidIdx_2])) {F2_P2 = true;}

            if (((F1_P1 || F2_P1) && (F1_P2 || F2_P2)) || (new_childs_try_count == 100*_inpop.solution.size())) {
            // if ((F1_P1 && F2_P1) || (new_childs_try_count == 100*_inpop.solution.size())) {
                _outpop.solution.push_back(_tmp_solution);
                // std::cout << "------------new_childs_try_count:" << new_childs_try_count << std::endl;
                // if (F1_P1) {std::cout << "------------F1_P1:" << idx << std::endl;}
                // if (F2_P1) {std::cout << "------------F2_P1:" << idx << std::endl;}
                // if (F1_P2) {std::cout << "------------F1_P2:" << idx << std::endl;}
                // if (F2_P2) {std::cout << "------------F2_P2:" << idx << std::endl;}
                if (new_childs_try_count != 100*_inpop.solution.size()) {
                    // std::cout << "------------GOOD CHILD CREATED!!:" << std::endl;
                } else {
                    std::cout << "------------LAST SOLUTION ADDED!!:" <<  idx << std::endl;
                }
            } else {
                // std::cout << "------------NEW CHILDS GENERATION!!:" <<  idx << std::endl;
                // crossCandidIdx_2 = crossCandidIdx_2 + 1;
                // if (crossCandidIdx_2 < _inpop.solution.size()) {
                //    goto AGAIN;
                // }
                computation_msgs::RAD_VEGA_SOLUTION _tmp_sol;
                autoware_msgs::Lane _tmp_lane;
                _tmp_sol.vehicleA.lanes.push_back(_tmp_lane);
                _tmp_sol.vehicleB.lanes.push_back(_tmp_lane);
                float coef = 0;
                coef = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
                for (auto wayPoint : _tmp_solution.vehicleA.lanes[0].waypoints) {
                    // wayPoint.twist.twist.linear.x = 0.1*(RAND(10, 0));
                    if (wayPoint.twist.twist.linear.x + coef > 1.0) {
                        wayPoint.twist.twist.linear.x = coef*abs(wayPoint.twist.twist.linear.x - coef);
                    } else {
                        wayPoint.twist.twist.linear.x = wayPoint.twist.twist.linear.x + coef;
                    }
                    // std::cout << "------------1HERE, idx:" << idx << std::endl;
                    // wayPoint.twist.twist.linear.x  = 0.5*wayPoint.twist.twist.linear.x; 
                    // std::cout << "A x: " << wayPoint.twist.twist.linear.x << std::endl;
                    // std::cout << "------------2HERE, wayPoint.twist.twist.linear.x:" << wayPoint.twist.twist.linear.x << std::endl;
                    _tmp_sol.vehicleA.lanes[0].waypoints.push_back(wayPoint);
                    // std::cout << "------------3HERE, idx:" << idx << std::endl;
                }
                _tmp_sol.vehicleA.lanes[0].waypoints[0].twist.twist.linear.x = coef;

                coef = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
                for (auto wayPoint : _tmp_solution.vehicleB.lanes[0].waypoints) {
                    // wayPoint.twist.twist.linear.x = 0.1*(RAND(10, 0));
                    if (wayPoint.twist.twist.linear.x + coef > 1.0) {
                        wayPoint.twist.twist.linear.x = coef*abs(wayPoint.twist.twist.linear.x - coef);
                    } else {
                        wayPoint.twist.twist.linear.x = wayPoint.twist.twist.linear.x + coef;
                    }
                    // wayPoint.twist.twist.linear.x = wayPoint.twist.twist.linear.x + (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
                    // wayPoint.twist.twist.linear.x  = 0.5*wayPoint.twist.twist.linear.x; 
                    // std::cout << "B x: " << wayPoint.twist.twist.linear.x << std::endl;
                    _tmp_sol.vehicleB.lanes[0].waypoints.push_back(wayPoint);
                }
                _tmp_sol.vehicleB.lanes[0].waypoints[0].twist.twist.linear.x = coef;
                _tmp_solution = _tmp_sol;
                new_childs_try_count = new_childs_try_count + 1;
                // std::cout << "------------not confirmed CHILD, idx:" << idx << std::endl;
                // std::cout << "------------not confirmed CHILD _tmp_solution.solution.size:" << _tmp_solution.vehicleA.lanes[0].waypoints.size() << std::endl;
                goto TRY_NEW_CHILDS;
            }
        } else if (_inpop.solution.size() > 1)  {
            // std::cout << "--------------------GOING TO NEXT CANDIDATE: " << idx << std::endl;
            goto NEXT_CANDIDATE;
        } else {
            // std::cout << "--------------------SING SOL reminde!: " << idx << std::endl;
        }
        // std::cout << "------ cross 3 idx: " << idx << std::endl;
        // std::cout << "------------2:_outpop.solution.size():" <<  _outpop.solution.size() << std::endl;

        // std::cout << "------ cross 4 idx" << idx << std::endl;
        // std::cout << "-----------------------out_f1: " << zdt1f1Objective(_tmp_solution) << ", out_f2: " << zdt1f2Objective(_tmp_solution) << std::endl;
        idx++;
    }
    // std::cout << "3:_outpop.solution.size():" <<  _outpop.solution.size() << std::endl;
    // }

    // while (_outpop.solution.size() < _inpop.solution.size()) {
    //     computation_msgs::RAD_VEGA_SOLUTION _tmp_sol;
    //     autoware_msgs::Lane _tmp_lane;
    //     _tmp_sol.vehicleA.lanes.push_back(_tmp_lane);
    //     _tmp_sol.vehicleB.lanes.push_back(_tmp_lane);
    //     for (int i = 0; i < 15; i++) {
    //         autoware_msgs::Waypoint _wp;
    //         // wayPoint.twist.twist.linear.x = 0.1*(RAND(10, 0));
    //         _wp.twist.twist.linear.x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    //         // std::cout << "A x: " << wayPoint.twist.twist.linear.x << std::endl;
    //         _tmp_sol.vehicleA.lanes[0].waypoints.push_back(_wp);
    //     }

    //     for (int i = 0; i < 15; i++) {
    //         autoware_msgs::Waypoint _wp;
    //         // wayPoint.twist.twist.linear.x = 0.1*(RAND(10, 0));
    //         _wp.twist.twist.linear.x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    //         // std::cout << "B x: " << wayPoint.twist.twist.linear.x << std::endl;
    //         _tmp_sol.vehicleB.lanes[0].waypoints.push_back(_wp);
    //     }
    //     _outpop.solution.push_back(_tmp_sol);
    // }
    // std::cout << "------ _outpop.solution.size(): " << _outpop.solution.size() << std::endl;
    return _outpop;
}

computation_msgs::RAD_VEGA_SOLUTION findBestSolution(computation_msgs::RAD_VEGA_POPULATION _pop) {
    computation_msgs::RAD_VEGA_SOLUTION _sol, _last_pushed_sol;
    computation_msgs::RAD_VEGA_POPULATION _pareto_frontier_set;
    _last_pushed_sol = _pop.solution.back();
    // int candIdx = 0;
    float MIN_COL_COST = 99999999;
    float MIN_POW_COST = 99999999;
    // Finding pareto frontier:
    // std::cout << "findBestSoloution, _pop.solution.size(): " << _pop.solution.size() << std::endl;
    for (int i = 0; i < _pop.solution.size(); i++) {
        // candIdx = RAND(_pop.solution.size(), 0);
        float _col_cost = zdt1f1Objective(_pop.solution[i]); 
        float _pow_cost = zdt1f2Objective(_pop.solution[i]); 
        // std::cout << "-----------------------it: " << i << ", _col_cost: " << _col_cost << ", _pow_cost: " << _pow_cost << std::endl;
        // std::cout << "-----_col_cost: " << _col_cost << std::endl;
        // std::cout << "-----_pow_cost: " << _pow_cost << std::endl;
        if ((zdt1f1Objective(_last_pushed_sol) != zdt1f1Objective(_pop.solution[i])
        && zdt1f2Objective(_last_pushed_sol) != zdt1f2Objective(_pop.solution[i])) 
        && ((_col_cost < MIN_COL_COST) || (_pow_cost < MIN_POW_COST))) {
            if ((_col_cost < MIN_COL_COST) && (_pow_cost < MIN_POW_COST)) {
                MIN_COL_COST = _col_cost;
                MIN_POW_COST = _pow_cost;
                _pareto_frontier_set.solution.clear();
                _pareto_frontier_set.solution.push_back(_pop.solution[i]);
                _last_pushed_sol = _pop.solution[i];
                // std::cout << "----_pareto_frontier_set.solution.size(): " << _pareto_frontier_set.solution.size() << std::endl;
                for (auto _sol : _pareto_frontier_set.solution) {
                    //std::cout << "-----------------------1_sol_f1: " << zdt1f1Objective(_sol) << std::endl;
                    //std::cout << "-----------------------1_sol_f2: " << zdt1f2Objective(_sol) << std::endl;
                }
                // _pop.solution.erase(_pop.solution.begin() + i);
            } else {
                _pareto_frontier_set.solution.push_back(_pop.solution[i]);
                _last_pushed_sol = _pop.solution[i];
                // std::cout << "----_pareto_frontier_set.solution.size(): " << _pareto_frontier_set.solution.size() << std::endl;
                
                // std::cout << "-----------------------: " << std::endl;
                // _pop.solution.erase(_pop.solution.begin() + i);
            }
        }
    }
    for (auto _sol : _pareto_frontier_set.solution) {
        std::cout << "f2: " << zdt1f2Objective(_sol) << std::endl;
        std::cout << "f1: " << zdt1f1Objective(_sol) << std::endl;
    }
    std::cout << "pareto_set_size: " << _pareto_frontier_set.solution.size() << std::endl;

    if (_pareto_frontier_set.solution.size() > 1) {
        int idx = RAND(_pareto_frontier_set.solution.size() - 1, 0);
        // std::cout << "-----_pop.solution.size(): " << _pop.solution.size() << std::endl;
        // std::cout << "-----_pareto_frontier_set.solution.size(): " << _pareto_frontier_set.solution.size() << std::endl;
        // std::cout << "-----idx: " << idx << std::endl;
        _sol = _pareto_frontier_set.solution[idx];
    }
    return _sol;
}

// std::vector<float> getFitnessVector(const computation_msgs::RAD_VEGA_POPULATION _population) {
//     int q = _population.solution.size() / 2;
//     std::vector<float> _fitness;
//     for (int j = 0; j < _population.solution.size(); j++) {
//         // std::cout << "------ RAD-VEGA Step 2. j: " << j << std::endl;
//         // std::cout << "------ _population.solution[j]: " << _population.solution[j].vehicleA.lanes[0].waypoints[0].twist.twist.linear.x << std::endl;
//         if (j < q) {
//             _fitness.push_back(zdt1f1Objective(_population.solution[j]));
//             // std::cout << "------ f1_fitness: " << _fitness.back() << std::endl;
//         } else if (j < _population.solution.size()) {
//             _fitness.push_back(zdt1f2Objective(_population.solution[j]));
//             // std::cout << "------ f2_fitness: " << _fitness.back() << std::endl;
//         }
//     }
//     return _fitness;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ZDT_solver_VEGA");
    ros::NodeHandle n;

    n.param<int>("/compute_model/FLOP_CALC_PERIOD_SEC", FLOP_CALC_PERIOD_SEC, 2);
    n.param<int>("/compute_model/LOOP_REPS", LOOP_REPS, 10000000);
    n.param<int>("/compute_model/N_p", N_p, 200);
    n.param<int>("/compute_model/APPCO", APPCO, 10);
    n.param<float>("/compute_model/c_vp_a", c_vp_a, 0.1);
    n.param<float>("/compute_model/c_vp_b", c_vp_b, 0.1);
    n.param<float>("/compute_model/c_ap_a", c_ap_a, 0.1);
    n.param<float>("/compute_model/c_ap_b", c_ap_b, 0.1);
    n.param<int>("/compute_model/t_min", t_min, 2000);

    status_pub = n.advertise<computation_msgs::status>("/computation/status", 1000);
    vega_stat_pub = n.advertise<computation_msgs::VEGA_stat>("/computation/VEGA_stat", 1000);
    cooperation_status_pub = n.advertise<cooperative_msgs::status>("/computation/cooperation_status", 1000);
    updated_lane_pub = n.advertise<autoware_msgs::LaneArray>("/computation/updated_lane_waypoints_array", 1000);
    computation_sub = n.subscribe("/rx_com", 1000, rxCallback);
    ego_path_sub = n.subscribe("/lane_waypoints_array", 1000, egoPathCallback);
    _vega_status.id = 1;
    _vega_status.IS_LEADER = false;
    _vega_status.CAV_total_flop = calcThisCAV_flp();
    // srand((unsigned) time(0));
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        _vega_status.CAV_flop = calcThisCAV_flp();
        _vega_status.CAV_t_available = calcThisCAV_t_available();
        _vega_stat.N_i = (N_p*_vega_status.CAV_flop*_vega_status.CAV_t_available) / (APPCO*_vega_status.CAV_total_flop*t_min);

        // RAD-VEGA Step 1:
        // std::cout << "------ RAD-VEGA Step 1. " << std::endl;
        computation_msgs::RAD_VEGA_POPULATION _population;
        int N_i = round(N_p*_vega_status.CAV_flop*_vega_status.CAV_t_available / (APPCO*_vega_status.CAV_total_flop*t_min));

        for (int i = 0; i < N_i; i++) {
            computation_msgs::RAD_VEGA_SOLUTION _tmp_sol;
            if (_solution.vehicleA.lanes.size() != 0 && _solution.vehicleB.lanes.size() != 0) {
                autoware_msgs::Lane _tmp_lane;
                _tmp_sol.vehicleA.lanes.push_back(_tmp_lane);
                _tmp_sol.vehicleB.lanes.push_back(_tmp_lane);
                for (auto wayPoint : _solution.vehicleA.lanes[0].waypoints) {
                    // wayPoint.twist.twist.linear.x = 0.1*(RAND(10, 0));
                    wayPoint.twist.twist.linear.x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                    // std::cout << "A x: " << wayPoint.twist.twist.linear.x << std::endl;
                    _tmp_sol.vehicleA.lanes[0].waypoints.push_back(wayPoint);
                }

                for (auto wayPoint : _solution.vehicleB.lanes[0].waypoints) {
                    // wayPoint.twist.twist.linear.x = 0.1*(RAND(10, 0));
                    wayPoint.twist.twist.linear.x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                    // std::cout << "B x: " << wayPoint.twist.twist.linear.x << std::endl;
                    _tmp_sol.vehicleB.lanes[0].waypoints.push_back(wayPoint);
                }
                _population.solution.push_back(_tmp_sol);
            }
        }

        // RAD-VEGA Step 2:
        // VEGA STEP 1: M = 2 (We are considering two objectives:
        // reduction in both collision avoidance risk and power consumption and navigation desire)
        // std::cout << "------ RAD-VEGA Step 2. " << std::endl;
        // std::vector<float> _fitness = getFitnessVector(_population);

        // Populating P_i
        // std::cout << "------ _population.solution.size(): "<< _population.solution.size() << std::endl;
        cout.setf(ios_base::fixed);
        clock_t rl_start = clock();
        // propSelection(_population);
        // _population = P_i;
        
        // std::cout << "------ RAD-VEGA Step 3.2. " << std::endl;
        cooperation_status_pub.publish(_coop_status);
        
        _vega_stat.partner_solution_population_size = P_vehicleB.solution.size();
        _vega_stat.this_solution_population_size = _population.solution.size();
        _vega_stat.total_solution_population_size = _population.solution.size() + P_vehicleB.solution.size();
        
        // std::cout << "------ _population.solution.size(): "<< _population.solution.size() << std::endl;
        // RAD-VEGA Step 3:
        _vega_status.P.solution.clear();
        _vega_status.P.solution.insert(_vega_status.P.solution.begin(), _population.solution.begin(), _population.solution.end());
        if (_vega_status.IS_LEADER && P_vehicleB.solution.size() > 0) {
            computation_msgs::RAD_VEGA_POPULATION P;
            P = _vega_status.P;
            // std::cout << "------ P.solution.size(): "<< P.solution.size() << std::endl;
            // std::cout << "------ AS parent. 1, P_vehicleB.solution.size()" << P_vehicleB.solution.size() << std::endl;
            // std::cout << "------ AS parent. 1.1, P.solution.size()" << P.solution.size() << std::endl;
            P.solution.insert(P.solution.end(), P_vehicleB.solution.begin(), P_vehicleB.solution.end());
            // std::cout << "------ +P.solution.size(): "<< P.solution.size() << std::endl;
            P_vehicleB.solution.clear();
            
            // std::cout << "-------P.size(): " << P.solution.size() << std::endl;
            int GA_IT = 500;
            for (int k = 0; k < GA_IT; k++) {
                // std::cout << "-------GENERATION: " << k << std::endl;
                // std::cout << "-------1_f1: " <<  zdt1f1Objective(P.solution.at(0)) << "-------1_f2: " <<  zdt1f2Objective(P.solution.at(0)) << std::endl;
                
                // std::cout << "-------2_f1: " <<  zdt1f1Objective(P.solution.at(0)) << "-------2_f2: " <<  zdt1f2Objective(P.solution.at(0)) << std::endl;
                // std::cout << "-------2: P.size(): " <<  P.solution.size() << std::endl;
                propSelection(P);
                if (P_i.solution.size() == 0) {
                    break;
                }
                P = P_i;
                // std::cout << "-------2.5: P.size(): " <<  P.solution.size() << std::endl;
                P = crossAndMate(P);
                
                // std::cout << "-------3: P.size(): " <<  P.solution.size() << std::endl;
                if (P.solution.size() == 0) {
                    P = P_i;
                    break;
                }
                
            }

            computation_msgs::RAD_VEGA_SOLUTION _sol = findBestSolution(P);
            // std::cout << "-------best_f1: " <<  zdt1f1Objective(_sol) << "-------best_f2: " <<  zdt1f2Objective(_sol) << std::endl;

            // std::cout << "------ ++P.solution.size(): "<< P.solution.size() << std::endl;
            // _vega_stat.total_solution_population_size = P.solution.size();
            // std::cout << "------ AS parent. 2 P.solution.size()" << P.solution.size() << std::endl;
            // std::cout << "------ AS parent. 2.1 VA Speed test" << P.solution[0].vehicleA.lanes[0].waypoints[0].twist.twist.linear.x << std::endl;
            // std::cout << "------ AS parent. 2.1 VB Speed test" << P.solution[0].vehicleB.lanes[0].waypoints[0].twist.twist.linear.x << std::endl;
            // std::cout << "------ AS parent. 2.1 zdt1f1Objective" << zdt1f1Objective(P.solution[0]) << std::endl;
            // std::cout << "------ AS parent. 2.1 zdt1f2Objective" << zdt1f2Objective(P.solution[0]) << std::endl;
            // std::cout << "------ AS parent. 3.1, _sol.vehicleA.lanes.size(): " << _sol.vehicleA.lanes.size() << std::endl;
            // std::cout << "------ AS parent. 3.1, _sol.vehicleB.lanes.size(): " << _sol.vehicleB.lanes.size() << std::endl;
            // computation_msgs::RAD_VEGA_SOLUTION _sol = findBestSolution(P_vehicleB);
            // std::cout << "------------------- P.solution.size(): " << P.solution.size() << std::endl;
            // std::cout << "------------------- P.solution.at(0).vehicleA.lanes.size(): " << P.solution.at(0).vehicleA.lanes.size() << std::endl;
            // std::cout << "------------------- P.solution.at(0).vehicleB.lanes.size(): " << P.solution.at(0).vehicleB.lanes.size() << std::endl;
            if (_sol.vehicleA.lanes.size() > 0 
            && _sol.vehicleB.lanes.size() > 0) {
                // std::cout << "------ AS parent. 3" << std::endl;
                _vega_stat.corrected_path_vehicleA = _sol.vehicleA;
                _vega_stat.corrected_path_vehicleB = _sol.vehicleB;
                // std::cout << "------ AS parent. 3.15 zdt1f1Objective(_sol): " << zdt1f1Objective(_sol) << std::endl;
                // std::cout << "------ AS parent. 3.15 zdt1f2Objective(_sol): " << zdt1f2Objective(_sol) << std::endl;
                _vega_stat.corrected_path_col_cost = zdt1f1Objective(_sol);
                _vega_stat.corrected_path_pow_cost = zdt1f2Objective(_sol);
                // std::cout << "------ AS parent. 3.2" << std::endl;
                _vega_stat.initial_path_vehicleA = _solution.vehicleA;
                _vega_stat.initial_path_vehicleB = _solution.vehicleB;
                _vega_stat.initial_path_col_cost = zdt1f1Objective(_solution);
                _vega_stat.initial_path_pow_cost = zdt1f2Objective(_solution);
            }
            // std::cout << "------ AS parent. 4" << std::endl;
            clock_t rl_end = clock();
            double rl_time = difftime(rl_end, rl_start) / CLOCKS_PER_SEC;
            std::cout << "-------EXE TIME: " <<  rl_time << std::endl;
            std::cout << "-------TOTAL Initial POP: " <<  _vega_stat.total_solution_population_size << std::endl;
            std::cout << "-------TOTAL Final POP: " <<  P.solution.size() << std::endl;
        }
        

        status_pub.publish(_vega_status);
        vega_stat_pub.publish(_vega_stat);
        ros::spinOnce();
        //ros::spin();
        loop_rate.sleep();
        // std::cout << "------ END. " << std::endl;
    }
    return 0;
}