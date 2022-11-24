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
#include "computation_msgs/NET_MEMBER.h"
#include "rosgraph_msgs/Clock.h"
#include <iostream>
#include <algorithm>
cooperative_msgs::status _coop_status;
computation_msgs::status _computation_status;
computation_msgs::RAD_VEGA_POPULATION P_i;
computation_msgs::RAD_VEGA_POPULATION _others_P;
computation_msgs::RAD_VEGA_SOLUTION _init_sol;

// computation_msgs::RAD_VEGA_SOLUTION _solution;
computation_msgs::VEGA_stat _vega_stat;
autoware_msgs::LaneArray _ego_path;
ros::Publisher status_pub, cooperation_status_pub, updated_lane_pub, vega_stat_pub;
ros::Subscriber computation_sub, ego_path_sub;

int FLOP_CALC_PERIOD_SEC;
int t_min = 1000; // Minimum time in msecond required a participant be available to contribute.
float LAST_FLOP_CALC_TIME;
int LOOP_REPS = 1000;
int N_p = 100;
float APPCO = 10.0;
int MAX_POP_SIZE = 300;
float c_vp_a, c_vp_b, c_ap_a, c_ap_b;
bool LEADER_EXIST = false;

using namespace std;
std::string NS;

float zdt1f1Objective(const computation_msgs::RAD_VEGA_SOLUTION _sol)
{
    return _sol.sol_set.at(0).lanes.at(0).waypoints.at(0).twist.twist.linear.x;
}

float zdt1f2Objective(const computation_msgs::RAD_VEGA_SOLUTION _sol)
{
    float sum;
    for (auto waypoint : _sol.sol_set.at(0).lanes.at(0).waypoints)
    {sum = sum + waypoint.twist.twist.linear.x;}
    sum = sum - _sol.sol_set.at(0).lanes.at(0).waypoints.at(0).twist.twist.linear.x;
    float g = sum*9.0/(_sol.sol_set[0].lanes.at(0).waypoints.size() - 1.0);
    g = g + 1;
    float _res;
    // std::cout << "---------------g: " << g << std::endl;
    _res = g*(1.0 - pow((_sol.sol_set[0].lanes.at(0).waypoints.at(0).twist.twist.linear.x/g),0.5));
    return _res;
}

void printPopFitness(const computation_msgs::RAD_VEGA_POPULATION _pop, std::string _title) {
    std::cout << "--------Fitnes of " << _title << " with size: " << _pop.solution.size() << " ----" << std::endl;
    std::cout << "--------------F1: " << std::endl;
    for (auto sol : _pop.solution) {
        std::cout << zdt1f1Objective(sol) << std::endl;
        // std::cout << "F2: " << zdt1f2Objective(sol) << std::endl;
        // std::cout << "--------------------------------------" << std::endl;
    }

    std::cout << "--------------F2: " << std::endl;
    for (auto sol : _pop.solution) {
        std::cout << zdt1f2Objective(sol) << std::endl;
    }
    std::cout << "---------------END of pop fitness print for " << _title << "-------" << std::endl;
}


computation_msgs::RAD_VEGA_POPULATION propSelection(computation_msgs::RAD_VEGA_POPULATION _pop)
{
    // We know that the half of the _pop should be evauated by f1
    // And the other half by f2 so:
    // if (NS == "vehicleA") {printPopFitness(_pop, " Proportionated Selection input ");}
    // while (_pop.solution.size() > 10000) {
    //     _pop.solution.pop_back();
    // }
    computation_msgs::RAD_VEGA_POPULATION _result;
    std::vector<float> _fitness_f1, _fitness_f2;
    computation_msgs::RAD_VEGA_POPULATION _pop_f1;
    computation_msgs::RAD_VEGA_POPULATION _pop_f2;
    computation_msgs::RAD_VEGA_POPULATION _sorted_pop_f1, _sorted_pop_f2;

    int SELECTED_POP_SIZE, NUMBER_OF_COPIES;
    // MAX_POP_SIZE = 0.9*_pop.solution.size();
    SELECTED_POP_SIZE = 20;
    if (_pop.solution.size() < SELECTED_POP_SIZE) {SELECTED_POP_SIZE = _pop.solution.size();} 

    for (int i = 0; i < _pop.solution.size(); i++)
    {
        if (i < _pop.solution.size() / 2)
        {
            _fitness_f1.push_back(zdt1f1Objective(_pop.solution.at(i)));
            _pop_f1.solution.push_back(_pop.solution.at(i));
        }
        else
        {
            _fitness_f2.push_back(zdt1f2Objective(_pop.solution.at(i)));
            _pop_f2.solution.push_back(_pop.solution.at(i));
        }
    }
    
    for (int j = 0; j < SELECTED_POP_SIZE/2; j++) {
        float fit_min = 99999.999;
        int fit_min_idx = 0;
        for (int i = 0; i < _fitness_f1.size(); i++) {
            if (_fitness_f1[i] < fit_min) {
                fit_min = _fitness_f1[i];
                fit_min_idx = i;
            }
        }
        _sorted_pop_f1.solution.push_back(_pop_f1.solution[fit_min_idx]);
        _fitness_f1.erase(_fitness_f1.begin() + fit_min_idx);
    }

    for (int j = 0; j < SELECTED_POP_SIZE/2; j++) {
        float fit_min = 99999.999;
        int fit_min_idx = 0;
        for (int i = 0; i < _fitness_f2.size(); i++) {
            if (_fitness_f2[i] < fit_min) {
                fit_min = _fitness_f2[i];
                fit_min_idx = i;
            }
        }
        _sorted_pop_f2.solution.push_back(_pop_f2.solution[fit_min_idx]);
        _fitness_f2.erase(_fitness_f2.begin() + fit_min_idx);
    }

    NUMBER_OF_COPIES = SELECTED_POP_SIZE;

    for (int k = 0; k < _sorted_pop_f1.solution.size(); k++) {
        for (int h = 0; h < NUMBER_OF_COPIES; h++) {
            _result.solution.push_back(_sorted_pop_f1.solution[k]);
            _result.solution.push_back(_sorted_pop_f2.solution[k]);
        }
        NUMBER_OF_COPIES = NUMBER_OF_COPIES - 1; 
    }

    // while (_result.solution.size() > MAX_POP_SIZE)
    // {
    //     _result.solution.pop_back();
    // }
    // if (NS == "vehicleA") {printPopFitness(_result, " Proportionated Selection output ");}

    return _result;
}

void rxCallback(communication_msgs::ComMessage msg)
{
    // std::cout << NS + ": --------------RX CALL BACK" << std::endl;
    // std::cout << NS + ": --------------msg->computation_status.CAV_flop" << msg->computation_status.CAV_flop << std::endl;
    // std::cout << NS + ": --------------_computation_status.CAV_flop" << _computation_status.CAV_flop << std::endl;
    // std::cout << NS + ": --------------msg->msg_source" << msg->msg_source << std::endl;
    

    if (msg.computation_status.CAV_flop != 0
    && _computation_status.CAV_flop != 0 && msg.msg_source != NS) {
        if (msg.computation_status.IS_LEADER) {
            LEADER_EXIST = true;
        }
        // Checking if new net member is sending message:
        bool NEW_MEMBER = true;
        for (auto member : _computation_status.net_member) {
            if (msg.msg_source == member.name) {
                NEW_MEMBER = false;
                break;
            }
        }

        // If new member joind net then add it to NET_MEMBERS with current time stamp
        if (NEW_MEMBER) {
            // std::cout << "From " << NS << ", New net member: " << msg.msg_source << std::endl;
            computation_msgs::NET_MEMBER _net_member;
            _net_member.name = msg.msg_source;
            _net_member.CAV_flop = msg.computation_status.CAV_flop;
            _net_member.CAV_available_time_ms = msg.computation_status.CAV_t_available;
            _net_member.header.stamp = ros::Time::now();
            _computation_status.net_member.push_back(_net_member);
        // Else just update the time stamp
        } else {
            for (int i = 0; i < _computation_status.net_member.size();i++) {
                if (msg.msg_source == _computation_status.net_member[i].name) {
                    // std::cout << "From " << NS << ", Update old net member: " << msg.msg_source << std::endl;
                    // std::cout << "From " << NS << ", _computation_status.net_member[i].name: " << _computation_status.net_member[i].name << std::endl;
                    // std::cout << "From " << NS << ", _computation_status.net_member[i].CAV_flop: " << _computation_status.net_member[i].CAV_flop << std::endl;
                    _computation_status.net_member[i].CAV_flop = msg.computation_status.CAV_flop;
                    _computation_status.net_member[i].CAV_available_time_ms = msg.computation_status.CAV_t_available;
                    _computation_status.net_member[i].header.stamp = ros::Time::now();
                }
            }
        }

        // if (_computation_status.IS_LEADER){std::cout << "--msg_source: " << msg.msg_source << "msg.computation_status.P.solution.size(): " << msg.computation_status.P.solution.size() << std::endl; }

        if(msg.computation_status.P.solution.size()>0) {
            _others_P.solution.insert(_others_P.solution.end(),
                                      msg.computation_status.P.solution.begin(),
                                      msg.computation_status.P.solution.end());
        }
    }
}

void chooseLeader(const ros::TimerEvent& event) {
    computation_msgs::NET_MEMBER _leader;
    _leader.CAV_flop = _computation_status.CAV_flop;
    _leader.name = NS;
    _leader.CAV_available_time_ms = _computation_status.CAV_t_available;
    _leader.role = "LEADER";
    for (int i = 0; i < _computation_status.net_member.size(); i++) {
        if (_computation_status.net_member[i].CAV_flop > _leader.CAV_flop) {
            _leader = _computation_status.net_member[i];
        }
    }
    if (_leader.name != NS) {
        _computation_status.IS_LEADER = false;
        for (int i = 0; i < _computation_status.net_member.size(); i++) {
            if (_leader.name ==_computation_status.net_member[i].name) {
                _computation_status.net_member[i].role = "LEADER";
            } else {
                _computation_status.net_member[i].role = "FOLLOWER";
            }
        }
    } else {
        if (!LEADER_EXIST) {_computation_status.IS_LEADER = true;}
        for (int i = 0; i < _computation_status.net_member.size(); i++) {
            _computation_status.net_member[i].role = "FOLLOWER";
        }
    }
    LEADER_EXIST = false;
}

void calcThisCAV_flp(const ros::TimerEvent& event)
{
    cout.setf(ios_base::fixed); // shows decimals in the output
    // cout << "loop_reps: " << LOOP_REPS << endl;
    // reference loop
    clock_t rl_start = clock();
    // loop index is volatile so that the empty loop isn't optimized away
    for (volatile uint32_t rl_index = 0; rl_index < LOOP_REPS; ++rl_index)
    {
        // empty loop - just to calculate how much time an empty loop _pareto_frontier_setneeds
    }
    clock_t rl_end = clock();
    double rl_time = difftime(rl_end, rl_start) / CLOCKS_PER_SEC;

    // output the time the reference loop took
    // cout << "cl_time:   " << rl_time << endl;
    // flops loop
    volatile float a = 1.5;
    volatile float b = 1.6;
    clock_t fl_start = clock();
    for (volatile uint32_t fl_index = 0; fl_index < LOOP_REPS; ++fl_index)
    {
        a *= b; // multiplication operation
        b += a; // addition operation
    }
    clock_t fl_end = clock();
    double fl_time = difftime(fl_end, fl_start) / CLOCKS_PER_SEC;
    unsigned long flops = LOOP_REPS / ((fl_time - rl_time) / 2);
    int _tmp = flops/1000000;
    _computation_status.CAV_flop = (float)_tmp;
    // std::cout << NS << ", " <<ros::Time::now() << ", " <<  _computation_status.CAV_flop << std::endl;
    // std::cout << NS << ", " <<ros::Time::now() << ", " <<  _computation_status.CAV_total_flop << std::endl;

}

void calcThisCAV_t_available(const ros::TimerEvent& event)
{
    _computation_status.CAV_t_available = 5000;
}

void checkNetMembers(const ros::TimerEvent& event) {
    float NET_MEMBERS_MAX_DELAY = 10;
    bool change_in_net = false;
    _computation_status.CAV_total_flop = 0;
    computation_msgs::NET_MEMBER _updated_net_members;
    for (int i = 0; i < _computation_status.net_member.size(); i++) {
        if ((ros::Time::now() - _computation_status.net_member[i].header.stamp).toSec() > NET_MEMBERS_MAX_DELAY) {
            _computation_status.net_member.erase(_computation_status.net_member.begin() + i);
            change_in_net = true;
        } else {
            _computation_status.CAV_total_flop = _computation_status.CAV_total_flop + _computation_status.net_member[i].CAV_flop; 
        }
    }
    _computation_status.CAV_total_flop = _computation_status.CAV_total_flop + _computation_status.CAV_flop;
    if (change_in_net){
        ros::TimerEvent _tev;
        _tev.current_expected = ros::Time::now();
        _tev.current_real = ros::Time::now();
        // calcThisCAV_flp(_tev);
        // calcThisCAV_t_available(_tev);
        chooseLeader(_tev);
    } 
}
void egoPathCallback(const autoware_msgs::LaneArray msg)
{
    // std::cout << "From " + NS + "ego path callback" << std::endl;
    _init_sol.name = NS;
    _init_sol.sol_set.clear();
    _init_sol.sol_set.push_back(msg);
    _init_sol.header.stamp = ros::Time::now();
    // if (P_i.solution.size() == 0) {P_i.solution.push_back(_sol);}
    // ego_path_sub.shutdown();
    if (P_i.solution.size() == 0){P_i.solution.push_back(_init_sol);}

    if (msg.lanes[0].waypoints.size() == _init_sol.sol_set[0].lanes[0].waypoints.size()) {
        for (int i = 0; i < msg.lanes.size(); i++) {
            if (msg.lanes[0].waypoints[i].twist.twist.linear.x != _init_sol.sol_set[0].lanes[0].waypoints[i].twist.twist.linear.x)
            {
                P_i.solution.clear();
                P_i.solution.push_back(_init_sol);
                _others_P.solution.clear();
                std::cout << "---REST INIT SOL----" << std::endl;
                break;
            }
        }
    } else {
        P_i.solution.clear();
        P_i.solution.push_back(_init_sol);
        std::cout << "---REST INIT SOL----" << std::endl;
        _others_P.solution.clear();
    }
}

int RAND(const int a, const int b)
{
    return b + (std::rand() % (a - b + 1));
}

computation_msgs::RAD_VEGA_POPULATION findParetoSet(const computation_msgs::RAD_VEGA_POPULATION _pop, bool _debug) {
    computation_msgs::RAD_VEGA_POPULATION _pareto_set;

    for (auto _sol : _pop.solution) {
        bool _pareto_member_found = false;
        for (int j = 0; j < _pop.solution.size(); j++) {
            if ((zdt1f1Objective(_pop.solution[j]) < zdt1f1Objective(_sol)
            && zdt1f2Objective(_pop.solution[j]) < zdt1f2Objective(_sol))
            || (zdt1f1Objective(_pop.solution[j]) == zdt1f1Objective(_sol)
            && zdt1f2Objective(_pop.solution[j]) < zdt1f2Objective(_sol))
            || (zdt1f1Objective(_pop.solution[j]) < zdt1f1Objective(_sol)
            && zdt1f2Objective(_pop.solution[j]) == zdt1f2Objective(_sol))) {
                _pareto_member_found = false;
                break;
            } else {_pareto_member_found = true;}
        }

        if (_pareto_member_found) {
            for (auto sol : _pareto_set.solution) {
                if(zdt1f1Objective(sol) == zdt1f1Objective(_sol)
                && zdt1f2Objective(sol) == zdt1f2Objective(_sol)) {
                    _pareto_member_found = false;
                    break;
                }
            }
        }

        if (_pareto_member_found) {
           _pareto_set.solution.push_back(_sol);
        }
    }

    if (_debug) {
        // std::cout << "------------PARTEO SET, F1: ------------" << std::endl;
        // for (auto _sol: _pareto_set.solution) {
        //     std::cout << zdt1f1Objective(_sol) << std::endl;
        // }
        // std::cout << "------------PARTEO SET, F2: ------------" << std::endl;
        // for (auto _sol: _pareto_set.solution) {
        //     std::cout << zdt1f2Objective(_sol) << std::endl;
        // }
        // printPopFitness(_pop, " Pareto set input ");
        printPopFitness(_pareto_set, " Pareto set output ");
    }
    return _pareto_set;
}

computation_msgs::RAD_VEGA_POPULATION crossAndMate(computation_msgs::RAD_VEGA_POPULATION _inpop) {
    computation_msgs::RAD_VEGA_POPULATION _outpop;
    int idx = 0;
    int cross_idx = 0;
    int crossCandidIdx_1 = 0;
    int crossCandidIdx_2 = 0;
    int new_child_try_max = 1*_inpop.solution.size();
    int MAX_MUT_COUNT = 1*_inpop.solution.size();
    int MAX_CROSS_COUNT = 5;
    
    // printPopFitness(_inpop, " CrossOver and Mutation input ");

    while (cross_idx < MAX_CROSS_COUNT) {
        _outpop.solution.clear();
        while (idx < MAX_MUT_COUNT) {
            computation_msgs::RAD_VEGA_SOLUTION _tmp_solution;
            NEXT_CANDIDATE:
            crossCandidIdx_1 = round((static_cast <float> (rand()) / static_cast <float> (RAND_MAX))*_inpop.solution.size());
            crossCandidIdx_2 = round((static_cast <float> (rand()) / static_cast <float> (RAND_MAX))*_inpop.solution.size());
    
            int _so_tmp = _inpop.solution.size() - 1;
            crossCandidIdx_1 = std::min(crossCandidIdx_1,_so_tmp);
            crossCandidIdx_2 = std::min(crossCandidIdx_2,_so_tmp);
            if (zdt1f1Objective(_inpop.solution[crossCandidIdx_1]) != zdt1f1Objective(_inpop.solution[crossCandidIdx_2])
            && zdt1f2Objective(_inpop.solution[crossCandidIdx_1]) != zdt1f2Objective(_inpop.solution[crossCandidIdx_2])) {
                autoware_msgs::LaneArray _lane_arr;
                autoware_msgs::Lane _lane;
                for (int i = 0; i < _inpop.solution[crossCandidIdx_1].sol_set.at(0).lanes.at(0).waypoints.size()/2;i++) {
                    autoware_msgs::Waypoint _wp;
                    _wp = _inpop.solution[crossCandidIdx_1].sol_set.at(0).lanes.at(0).waypoints[i];
                    // if(_computation_status.IS_LEADER){std::cout << NS << ", child x: " << _wp.twist.twist.linear.x << std::endl;}
                    _lane.waypoints.push_back(_wp);
                }

                for (int i = (_inpop.solution[crossCandidIdx_1].sol_set.at(0).lanes.at(0).waypoints.size()/2); i < _inpop.solution[crossCandidIdx_1].sol_set.at(0).lanes.at(0).waypoints.size();i++) {
                    autoware_msgs::Waypoint _wp;
                    _wp = _inpop.solution[crossCandidIdx_2].sol_set.at(0).lanes.at(0).waypoints[i];
                    // if(_computation_status.IS_LEADER){std::cout << NS << ", child x: " << _wp.twist.twist.linear.x << std::endl;}
                    _lane.waypoints.push_back(_wp);
                }

                // if(_computation_status.IS_LEADER) {
                //     for (int i = 0; i < _inpop.solution[crossCandidIdx_1].sol_set[0].lanes[0].waypoints.size(); i++) {
                //         std::cout << NS << ", parent 1 x: " << _inpop.solution[crossCandidIdx_1].sol_set[0].lanes[0].waypoints[i].twist.twist.linear.x << std::endl;
                //     }

                //     for (int i = 0; i < _inpop.solution[crossCandidIdx_2].sol_set[0].lanes[0].waypoints.size(); i++) {
                //         std::cout << NS << ", parent 2 x: " << _inpop.solution[crossCandidIdx_2].sol_set[0].lanes[0].waypoints[i].twist.twist.linear.x << std::endl;
                //     }
                // }   
                _lane_arr.lanes.push_back(_lane);
                _tmp_solution.sol_set.push_back(_lane_arr);
                int new_childs_try_count = 0;

                TRY_NEW_CHILDS:
                // bool F1_P1 = false;
                // bool F1_P2 = false;
                // bool F2_P1 = false;
                // bool F2_P2 = false;
                // if (zdt1f1Objective(_tmp_solution) < zdt1f1Objective(_inpop.solution[crossCandidIdx_1])) {F1_P1 = true;}
                // if (zdt1f1Objective(_tmp_solution) < zdt1f1Objective(_inpop.solution[crossCandidIdx_2])) {F1_P2 = true;}
                // if (zdt1f2Objective(_tmp_solution) < zdt1f2Objective(_inpop.solution[crossCandidIdx_1])) {F2_P1 = true;}
                // if (zdt1f2Objective(_tmp_solution) < zdt1f2Objective(_inpop.solution[crossCandidIdx_2])) {F2_P2 = true;}
                computation_msgs::RAD_VEGA_POPULATION _tmp_pop, _tmp_pareto;
                _tmp_pop.solution.push_back(_tmp_solution);
                _tmp_pop.solution.push_back(_inpop.solution[crossCandidIdx_1]);
                _tmp_pop.solution.push_back(_inpop.solution[crossCandidIdx_2]);
                _tmp_pareto = findParetoSet(_tmp_pop, false);
                bool child_inside = false;
                for (auto _sol : _tmp_pareto.solution) {
                    if (_sol == _tmp_solution) {
                        child_inside = true;
                        break;
                    } else {
                        // if(_computation_status.IS_LEADER){std::cout << NS << ", sol at pareto found " << std::endl;}
                    }
                }
                // child_inside = true;
                // if(_computation_status.IS_LEADER && child_inside){std::cout << NS << ", child Inside: "  << ", at: "<< idx << std::endl;}
                if (_tmp_pareto.solution.size()>0 && child_inside) {
                    // if(_computation_status.IS_LEADER){std::cout << NS << ", _tmp_pareto.solution.size(): " << _tmp_pareto.solution.size() << ", at: "<< idx << std::endl;}
                // if ((F1_P1 && F2_P1) && (F1_P2 && F2_P2)) {
                // if (F1_P1 || F1_P2 || F2_P1 || F2_P2) {
                    for (auto _sol : _tmp_pareto.solution) {
                        // if(_computation_status.IS_LEADER){std::cout << NS << ", sol at pareto found " << std::endl;}
                        _outpop.solution.push_back(_sol);
                        idx++;
                    }
                    
                } else if (new_childs_try_count != new_child_try_max) {
                    float coef;
                    for (int i = 0; i < _tmp_solution.sol_set[0].lanes[0].waypoints.size(); i++) {
                        coef = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
                        if (coef < 0.5) {
                            _tmp_solution.sol_set[0].lanes[0].waypoints[i].twist.twist.linear.x = 1.0 - _tmp_solution.sol_set[0].lanes[0].waypoints[i].twist.twist.linear.x;
                        }
                    }
                    new_childs_try_count = new_childs_try_count + 1;
                    goto TRY_NEW_CHILDS;
                } else {
                    // if(_computation_status.IS_LEADER){std::cout << NS << ", parents add at: " << idx << std::endl;}
                    _outpop.solution.push_back(_inpop.solution[crossCandidIdx_1]);
                    _outpop.solution.push_back(_inpop.solution[crossCandidIdx_2]);
                    idx = idx + 2;
                    // if (_outpop.solution.size() > 10000) {idx = MAX_MUT_COUNT;}
                }
            } else if (_inpop.solution.size() > 1)  {
                goto NEXT_CANDIDATE;
            }
        }
        cross_idx++;
        _inpop = _outpop;
        idx = 0;
    }
    // printPopFitness(_outpop, " CrossOver and Mutation output ");
    return _outpop;
 }

computation_msgs::RAD_VEGA_SOLUTION findBestSolution(computation_msgs::RAD_VEGA_POPULATION _pop) {
    computation_msgs::RAD_VEGA_SOLUTION _sol;
    if (_pop.solution.size() > 1) {
        int idx = RAND(_pop.solution.size() - 1, 0);
        _sol = _pop.solution[idx];
    }
    return _sol;
}

float Dg(const computation_msgs::RAD_VEGA_POPULATION _pop) {
    float _result = 0;
    for (int i = 0; i < _pop.solution.size(); i++) {
        float _f1 = zdt1f1Objective(_pop.solution[i]);
        if(_f1 < 0.01) 
        {_result = _result + pow(1 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.05)
        {_result = _result + pow(0.77 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.1)
        {_result = _result + pow(0.67 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.15)
        {_result = _result + pow(0.6 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.21)
        {_result = _result + pow(0.54 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.26)
        {_result = _result + pow(0.48 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.31)
        {_result = _result + pow(0.43 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.37)
        {_result = _result + pow(0.39 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.42)
        {_result = _result + pow(0.35 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.47)
        {_result = _result + pow(0.31 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.52)
        {_result = _result + pow(0.27 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.58)
        {_result = _result + pow(0.24 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.63)
        {_result = _result + pow(0.2 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.68)
        {_result = _result + pow(0.17 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.73)
        {_result = _result + pow(0.14 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.79)
        {_result = _result + pow(0.11 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.84)
        {_result = _result + pow(0.08 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.89)
        {_result = _result + pow(0.05 - zdt1f2Objective(_pop.solution[i]),2);}
        else if (_f1 < 0.94)
        {_result = _result + pow(0.02 - zdt1f2Objective(_pop.solution[i]),2);}
        else {_result = _result + pow(- zdt1f2Objective(_pop.solution[i]),2);}
    }
    return _result/_pop.solution.size();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ZDT_solver_VEGA");
    ros::NodeHandle n;
    // std::cout << "-----------------------ARGC: " << argc << std::endl;
    // std::cout << "-----------------------ARGV: " << argv[argc-1] << std::endl;
    NS = argv[argc-1];

    n.param<int>("/ZDT_solver_VEGA/FLOP_CALC_PERIOD_SEC", FLOP_CALC_PERIOD_SEC, 2);
    n.param<int>("/ZDT_solver_VEGA/LOOP_REPS", LOOP_REPS, 10000000);
    n.param<int>("/ZDT_solver_VEGA/N_p", N_p, 100);
    n.param<float>("/ZDT_solver_VEGA/APPCO", APPCO, 0.0001);
    n.param<float>("/ZDT_solver_VEGA/c_vp_a", c_vp_a, 0.1);
    n.param<float>("/ZDT_solver_VEGA/c_vp_b", c_vp_b, 0.1);
    n.param<float>("/ZDT_solver_VEGA/c_ap_a", c_ap_a, 0.1);
    n.param<float>("/ZDT_solver_VEGA/c_ap_b", c_ap_b, 0.1);
    n.param<int>("/ZDT_solver_VEGA/t_min", t_min, 10.0);

    status_pub = n.advertise<computation_msgs::status>("/computation/status", 1);
    vega_stat_pub = n.advertise<computation_msgs::VEGA_stat>("/computation/VEGA_stat", 1);
    cooperation_status_pub = n.advertise<cooperative_msgs::status>("/computation/cooperation_status", 1);
    updated_lane_pub = n.advertise<autoware_msgs::LaneArray>("/computation/updated_lane_waypoints_array", 1);
    computation_sub = n.subscribe("/rx_com", 1, rxCallback);
    ego_path_sub = n.subscribe("/lane_waypoints_array", 1, egoPathCallback);

    ros::Timer timer_FLOP_CALC = n.createTimer(ros::Duration(300.0), calcThisCAV_flp);
    ros::Timer timer_EAT_CALC = n.createTimer(ros::Duration(300.0), calcThisCAV_t_available);
    ros::Timer timer_leader_selection = n.createTimer(ros::Duration(2.0), chooseLeader);
    ros::Timer timer_check_net_members = n.createTimer(ros::Duration(1.0), checkNetMembers);
    _computation_status.id = 1;
    _computation_status.IS_LEADER = false;
    ros::Rate loop_rate(10);

    ros::TimerEvent _tev;
    _tev.current_expected = ros::Time::now();
    _tev.current_real = ros::Time::now();
    calcThisCAV_flp(_tev);
    calcThisCAV_t_available(_tev);
    chooseLeader(_tev);
    int OTHERS_P_MAX_SIZE = 100;
    while (ros::ok())
    {
        // while (_others_P.solution.size() > OTHERS_P_MAX_SIZE) {
        //     _others_P.solution.pop_back();
        // }
        if (_computation_status.CAV_total_flop != 0 && _computation_status.CAV_t_available != 0) {
            _vega_stat.N_i = N_p*_computation_status.CAV_flop*_computation_status.CAV_t_available*APPCO*t_min/_computation_status.CAV_total_flop;
            int N_i = _vega_stat.N_i;
            while (_others_P.solution.size() > N_i) {
                _others_P.solution.pop_back();
            }
            // std::cout << "-------_computation_status.CAV_flop: " << _computation_status.CAV_flop << std::endl;
            // std::cout << "-------_computation_status.CAV_t_available: " <<_computation_status.CAV_t_available << std::endl;
            // std::cout << "-------APPCO: " <<APPCO << std::endl;
            // std::cout << "-------t_min: " <<t_min << std::endl;
            // std::cout << "-------down: " << _computation_status.CAV_total_flop << std::endl;
            // std::cout << "-------N_i: " << N_i << std::endl;
            // std::cout << "-------_vega_stat.N_i: " << _vega_stat.N_i << std::endl;
            if (P_i.solution.size() == 1) {
                for (int i = 0; i < N_i - _others_P.solution.size(); i++) {
                    computation_msgs::RAD_VEGA_SOLUTION _tmp_sol;
                    _tmp_sol = P_i.solution.at(0);
                    autoware_msgs::Lane _tmp_lane;
                    // float _coef = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                    // if (_coef > 0.5) {   
                    //     for (auto wayPoint : _tmp_sol.sol_set.at(0).lanes.at(0).waypoints) {
                    //         wayPoint.twist.twist.linear.x = _coef*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
                    //         _tmp_lane.waypoints.push_back(wayPoint);
                    //     }
                    // } else {
                    //     for (auto wayPoint : _tmp_sol.sol_set.at(0).lanes.at(0).waypoints) {
                    //         wayPoint.twist.twist.linear.x = 1 - (_coef*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX)));
                    //         _tmp_lane.waypoints.push_back(wayPoint);
                    //     }
                    // }
                    for (auto wayPoint : _tmp_sol.sol_set.at(0).lanes.at(0).waypoints) {
                        // float _coef = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                        // if (_coef < 0.333) {
                            // wayPoint.twist.twist.linear.x = 1 - (_coef*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX)));
                        // } else if (_coef < 0.666){
                        wayPoint.twist.twist.linear.x = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
                        // } else {
                            // wayPoint.twist.twist.linear.x = _coef*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
                        // }
                        _tmp_lane.waypoints.push_back(wayPoint);
                    }
                    _tmp_sol.sol_set.at(0).lanes.at(0) = _tmp_lane;
                    P_i.solution.push_back(_tmp_sol);
                }
                // std::cout << "-------Final P_i.solution.size(): " << P_i.solution.size() << std::endl;
            }
            _vega_stat.NoOfAgents = _computation_status.net_member.size() + 1;
        }
        
        int GA_IT = 5;
        if (_others_P.solution.size() == 0) {GA_IT = 1;}

        if (P_i.solution.size() > 1) {
            P_i.solution.insert(P_i.solution.end(), _others_P.solution.begin(), _others_P.solution.end());
            cout.setf(ios_base::fixed);
            clock_t rl_start = clock();
            computation_msgs::RAD_VEGA_POPULATION _tmp_pop;
            for (int k = 0; k < GA_IT; k++) {
                // if(_computation_status.IS_LEADER){std::cout << NS << ", ------- GEN: " << k << std::endl;}
                // if(_computation_status.IS_LEADER){std::cout  << NS << ", ------- GEN: " << k << ", P_i.solution.size(): " << P_i.solution.size() << std::endl;}
                // if (_computation_status.IS_LEADER){printPopFitness(P_i, " P_i ");}

                _tmp_pop = P_i;
                P_i = crossAndMate(P_i);
                // if(_computation_status.IS_LEADER){std::cout << "-------After propSel P_i.solution.size(): " << P_i.solution.size() << std::endl;}
                // if (_computation_status.IS_LEADER){printPopFitness(P_i, " P_i after propSel");}
                // std::cout << "------- Proportionate selection, P_i.solution.size(): " << P_i.solution.size() << std::endl;
                if (P_i.solution.size() == 0) {
                    P_i = _tmp_pop;
                    if(_computation_status.IS_LEADER){std::cout << "-------WOOPS----------- " << std::endl;}
                    break;
                }
                _tmp_pop = P_i;
                P_i = propSelection(P_i);
                // if(_computation_status.IS_LEADER){std::cout << "-------After cross P_i.solution.size(): " << P_i.solution.size() << std::endl;}
                if (P_i.solution.size() == 0) {
                    P_i = _tmp_pop;
                    if(_computation_status.IS_LEADER){std::cout << "-------WOOPS----------- " << std::endl;}
                    break;
                }
                // if (_computation_status.IS_LEADER){printPopFitness(P_i, " P_i after crossAndMut");}
                // std::cout << "------- CrossOverAndMutation, P_i.solution.size(): " << P_i.solution.size() << std::endl;
            }
            // OTHERS_P_MAX_SIZE = 100*(_computation_status.net_member.size() + 1);
            computation_msgs::RAD_VEGA_POPULATION _pareto_set;
            P_i.solution.insert(P_i.solution.end(), _others_P.solution.begin(), _others_P.solution.end());

            // if(_computation_status.IS_LEADER){_pareto_set = findParetoSet(P_i, false);}
            // else {findParetoSet(P_i, false);}
            _pareto_set = findParetoSet(P_i, false);
            // if(_computation_status.IS_LEADER){std::cout << "---P_i.solution.size(): " << P_i.solution.size() << std::endl;}
            // if(_computation_status.IS_LEADER){std::cout << "---_pareto_set.solution.size(): " << _pareto_set.solution.size() << std::endl;}
            
            // if(_computation_status.IS_LEADER){std::cout << "-------First findParetoSet END---------" << std::endl;}
            _computation_status.P = _pareto_set;

            // if(_computation_status.IS_LEADER){std::cout << ros::Time::now() << ", " <<  _computation_status.CAV_flop << std::endl;}
            if (_computation_status.IS_LEADER) {
                // computation_msgs::RAD_VEGA_POPULATION _final_pareto_set;
                // _final_P = P_i;
                // std::cout << "-----_FROM " << NS << ", THE ONLY LEADER-------------" << std::endl;
                // std::cout << "P_i.size(): " << P_i.solution.size() << std::endl;
                // std::cout << "_final_P.size(): " << _final_P.solution.size() << std::endl;
                // _final_P.solution.insert(_final_P.solution.end(), _others_P.solution.begin(), _others_P.solution.end());
                // std::cout << "_final_P.size(): " << _final_P.solution.size() << std::endl;
                // _final_pareto_set = findParetoSet(_final_P, true);
                // if (_others_P.solution.size()>0){printPopFitness(_pareto_set, " From leader (" + NS + ") Pareto set: ");}
                // _computation_status.P = _final_pareto_set;
                computation_msgs::RAD_VEGA_SOLUTION _best_sol = findBestSolution(_pareto_set);
                clock_t rl_end = clock();
                double rl_time = difftime(rl_end, rl_start) / CLOCKS_PER_SEC;
                // std::cout << "From " << NS << "-------RESULTS:------------" << std::endl;
                // std::cout << "From " << NS << "-------_pareto_set.solution.size(): " <<  _pareto_set.solution.size() << std::endl;
                // std::cout << "From " << NS << "-------Cooperative participant solutions: " <<  _others_P.solution.size() << std::endl;
                // std::cout << "From " << NS << "-------local solutions: " << P_i.solution.size() - _others_P.solution.size() << std::endl;
                // std::cout << "From " << NS << "-------Total Generation iterrations: " << GA_IT << std::endl;
                // std::cout << "From " << NS << "-------SIM TIME: " <<  ros::Time::now() << std::endl;
                // std::cout << "From " << NS << "-------EXE TIME: " <<  rl_time << std::endl;
                // std::cout << "From " << NS << "-------Dg: " <<  Dg(_pareto_set) << std::endl;
                // std::cout << "From " << NS << "----------------------------------------------" << std::endl;
                // std::cout << ros::Time::now() << ", " << Dg(_pareto_set) << std::endl;
                // return 0;
            } else {
                // std::cout << NS << " is not ready" << std::endl;
            }

            P_i.solution.clear();
            P_i.solution.push_back(_init_sol);
            // _others_P.solution.clear();
        }
        

        // ego_path_sub = n.subscribe("/lane_waypoints_array", 1, egoPathCallback);
        // std::cout << NS <<  ", " << ros::Time::now() << ", " <<_computation_status.CAV_flop << std::endl;
        status_pub.publish(_computation_status);
        vega_stat_pub.publish(_vega_stat);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}