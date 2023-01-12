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
#include <numeric>
#include <string_view>
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
int CAV_total_flop = 0;

const float min_distance = 0.5;
using namespace std;
std::string NS;

float powObjective(const computation_msgs::RAD_VEGA_SOLUTION _sol) {
    float H = 0;
    float c_a = 0.25;
    float c_v = 1.0;
    std::vector<float> pow_cost;
    for (auto sol : _sol.sol_set) {
        std::vector<float> acc, vel;
        for (int i = 0; i < sol.lanes[0].waypoints.size(); i++) {
            vel.push_back(sol.lanes[0].waypoints[i].twist.twist.linear.x);
            float _v = sol.lanes[0].waypoints[i].twist.twist.linear.x;
            float _x = sol.lanes[0].waypoints[i].pose.pose.position.x;
            float _y = sol.lanes[0].waypoints[i].pose.pose.position.y;
            
            float _v0 = 0.0;
            float _x0 = 0.0;
            float _y0 = 0.0;
            if (i > 0) {
                float _v0 = sol.lanes[0].waypoints[i-1].twist.twist.linear.x;
                float _x0 = sol.lanes[0].waypoints[i-1].pose.pose.position.x;
                float _y0 = sol.lanes[0].waypoints[i-1].pose.pose.position.y;
            }
            float _d = pow(pow(_x-_x0,2) + pow(_y-_y0,2),0.5);
            float _ac = 0;
            if (_d != 0) {_ac = (pow(_v,2)-pow(_v0,2))/(2.0*(abs(_d)));}
            acc.push_back(_ac);
        }
        pow_cost.push_back(c_a*std::accumulate(acc.begin(), acc.end(), 0.0) + c_v*std::accumulate(vel.begin(), vel.end(), 0.0));
    }
    H = std::accumulate(pow_cost.begin(), pow_cost.end(), 0.0);
    return H;
}

float assigneColRiskAtTime(
    autoware_msgs::LaneArray _sol1, 
    autoware_msgs::LaneArray _sol2,
    float T 
) {
    float t_sim = 0;
    int idx1, idx2;
    // std::cout << "_sol1.lanes[0].waypoints.size(): " << _sol1.lanes[0].waypoints.size() << std::endl;
    // std::cout << "_sol2.lanes[0].waypoints.size(): " << _sol2.lanes[0].waypoints.size() << std::endl;
    for (int i = 1; i < _sol1.lanes[0].waypoints.size(); i++) {
        float dx = abs(
            _sol1.lanes[0].waypoints[i].pose.pose.position.x
            - _sol1.lanes[0].waypoints[i-1].pose.pose.position.x
        );
        float dy = abs(
            _sol1.lanes[0].waypoints[i].pose.pose.position.y
            - _sol1.lanes[0].waypoints[i-1].pose.pose.position.y
        );
        float v0 = _sol1.lanes[0].waypoints[i-1].twist.twist.linear.x;
        float v = _sol1.lanes[0].waypoints[i].twist.twist.linear.x;
        float dd = pow(pow(dx,2) + pow(dy,2), 0.5);
        float dv = abs(v - v0);
        // if (v == 0) {std::cout << "waypoint idx: " << i << NS << ": -----------NOT COOL----------" << std::endl;}
        if (dv == 0) {
            if (v0 != 0) {t_sim = t_sim + dd/v0;}
            else if (v != 0) {t_sim = t_sim + dd/v;}
            // else {break;}
        } else {
            t_sim = t_sim + dd/dv;
        }
        // std::cout << NS << ", first t_sim: " << t_sim << std::endl;
        if (t_sim >= T) {
            idx1 = i - 1;
            break;
        }
    }

    t_sim = 0.0;
    for (int i = 1; i < _sol2.lanes[0].waypoints.size(); i++) {
        float dx = abs(
            _sol2.lanes[0].waypoints[i].pose.pose.position.x
            - _sol2.lanes[0].waypoints[i-1].pose.pose.position.x
        );
        float dy = abs(
            _sol2.lanes[0].waypoints[i].pose.pose.position.y
            - _sol2.lanes[0].waypoints[i-1].pose.pose.position.y
        );
        float v0 = _sol2.lanes[0].waypoints[i-1].twist.twist.linear.x;
        float v = _sol2.lanes[0].waypoints[i].twist.twist.linear.x;
        float dd = pow(pow(dx,2) + pow(dy,2), 0.5);
        float dv = abs(v - v0);
        // if (v == 0) {std::cout << "waypoint idx: " << i << NS << ": -----------NOT COOL----------" << std::endl;}
        if (dv == 0) {
            if (v0 != 0) {t_sim = t_sim + dd/v0;}
            else if (v != 0) {t_sim = t_sim + dd/v;}
            // else {break;}
        } else {
            t_sim = t_sim + dd/dv;
        }
        // std::cout << NS << ", second t_sim: " << t_sim << std::endl;
        // std::cout << NS << ", sol2: " << t_sim << std::endl;
        if (t_sim >= T) {
            idx2 = i - 1;
            break;
        }
    }

    // std::cout << NS << " s1 and s2: " << _sol1.lanes[0].waypoints.size() <<  ", "  << _sol2.lanes[0].waypoints.size() << std::endl;
    // std::cout << NS << " idx1 and idx2: " << idx1 <<  ", "  << idx2 << std::endl;
    // std::cout << NS << " idx1 and idx2: " << idx1 <<  ", "  << idx2 << std::endl;
    float distance = pow(pow(abs(
        _sol1.lanes[0].waypoints[idx1].pose.pose.position.x
        - _sol2.lanes[0].waypoints[idx2].pose.pose.position.x),2)
        + pow(abs(
        _sol1.lanes[0].waypoints[idx1].pose.pose.position.y
        - _sol2.lanes[0].waypoints[idx2].pose.pose.position.y),2),0.5);

    // if (_res == 0) {std::cout << NS << ": DISTANCE ZERO at sol1 and sol2 ids: " << _sol1.id << ", " << _sol2.id << std::endl;}
    // std::cout << NS << "------------_sol2.lanes[0].waypoints.size(): " << _sol2.lanes[0].waypoints.size() << std::endl;
    // std::cout << NS << "------------_sol1.lanes[0].waypoints.size(): " << _sol1.lanes[0].waypoints.size() << std::endl;
    float _res = 0;
    if (distance < min_distance){_res = 300.0/min_distance;}
    else {_res = 300.0/distance;}
    // else {_res = std::numeric_limits<float>::max()/std::max(
    //     _sol1.lanes[0].waypoints.size(), _sol2.lanes[0].waypoints.size());}
    if (_res == 0) {
        std::cout << NS << "------------ZERO COL RISK at time:  " << T << ", ids: " << _sol1.id << ", " << _sol2.id << _res << std::endl;
    } else {
        // std::cout << NS << ", col cost: " << _res << std::endl;
    }
    return _res;

}

void addToInitSol(const autoware_msgs::LaneArray path) {
    if (path.id != 0) {
        bool duplicated = false;
        for (auto _path : _init_sol.sol_set) {
            if (_path.id == path.id) {
                duplicated = true;
                break;
            }
        }
        if (!duplicated){_init_sol.sol_set.push_back(path);}
        // std::cout << NS << ": -----------_init_sol.sol_set: " << _init_sol.sol_set.size() << std::endl;
    }
}

float estimateRouteExeTime(const computation_msgs::RAD_VEGA_SOLUTION _sol) {
    float _res = 0.0;
    for (int i = 0; i < _sol.sol_set.size(); i++) {
        float distance, speed, time;
        for (int j = 1; j < _sol.sol_set[i].lanes[0].waypoints.size(); j++) {
            distance = distance + pow(
                pow(_sol.sol_set[i].lanes[0].waypoints[j].pose.pose.position.x
                - _sol.sol_set[i].lanes[0].waypoints[j-1].pose.pose.position.x,2)
                + pow(_sol.sol_set[i].lanes[0].waypoints[j].pose.pose.position.y
                - _sol.sol_set[i].lanes[0].waypoints[j-1].pose.pose.position.y,2),0.5);
            speed = speed + _sol.sol_set[i].lanes[0].waypoints[j].twist.twist.linear.x;
        }
        time = distance/((speed + _sol.sol_set[i].lanes[0].waypoints[0].twist.twist.linear.x)/_sol.sol_set[i].lanes[0].waypoints.size());
        _res = std::max(_res, time);
    }
    // std::cout << "-------------------_RES: " << _res << std::endl;
    return _res;
}

float colObjective(const computation_msgs::RAD_VEGA_SOLUTION _sol)
{
    float H = 0;
    float deltaT = 10; // seconds
    float route_exe_time = estimateRouteExeTime(_sol);
    float route_exe_time_step = route_exe_time/10.0;
    deltaT = std::min(deltaT, route_exe_time_step);
    std::vector<float> col_cost;
    for (auto sol1 : _sol.sol_set) {
        // std::cout << NS << "sol1.size(): " << sol1.lanes[0].waypoints.size() << std::endl;
        std::vector<float> D_sol;
        for (auto sol2 : _sol.sol_set) {
            if (sol1.id != sol2.id) {
                // std::cout << NS << "sol2.size(): " << sol2.lanes[0].waypoints.size() << std::endl;
                std::vector<float> D;
                float T = 0.0;
                while (T < route_exe_time) {
                    T = T + deltaT;
                    // std::cout << "---T: " << T << std::endl;
                    D.push_back(assigneColRiskAtTime(sol1,sol2,T));
                }
                if (D.size() == 0) {std::cout << " ------------- HERE 1------" << std::endl;}
                float _D_sum = std::accumulate(D.begin(), D.end(), 0.0);
                if (_D_sum == 0) {std::cout << " ------------- HERE 2------" << std::endl;}
                D_sol.push_back(_D_sum);
            }
        }
        if (D_sol.size() == 0) {
            // std::cout << " _sol.sol_set.size(): " << _sol.sol_set.size() << std::endl;
        } else {
            col_cost.push_back(std::accumulate(D_sol.begin(), D_sol.end(), 0.0));
        }
        // std::cout << NS << ": 5" << std::endl;
    }
    // if (col_cost.size() == 0) {
    //     std::cout << " ------------- HERE 3------" << std::endl;
    //     std::cout << "-------_sol.sol_set.size(): " << _sol.sol_set.size() << std::endl;
    //     for (auto s : _sol.sol_set) {
    //         std::cout << "-------id: " << s.id << std::endl;
    //     }
    // }
    for (auto cost : col_cost) {
        // std::cout << NS << " cost: " << cost << std::endl;
    }
    H = std::accumulate(col_cost.begin(), col_cost.end(), 0.0);
    // if (H == 0) {std::cout << " ------------- HERE 4------" << std::endl;}
    H = 10000.0;

    return H;
}

// void printPopFitness(const computation_msgs::RAD_VEGA_POPULATION _pop, std::string _title) {
//     std::cout << "--------Fitnes of " << _title << " with size: " << _pop.solution.size() << " ----" << std::endl;
//     std::cout << "--------------power consumption: " << std::endl;
//     for (auto sol : _pop.solution) {
//         std::cout << powObjective(sol) << std::endl;
//     }

//     std::cout << "--------------collision risk: " << std::endl;
//     for (auto sol : _pop.solution) {
//         std::cout << colObjective(sol) << std::endl;
//     }
//     std::cout << "---------------END of pop fitness print for " << _title << "-------" << std::endl;
// }

void printPopFitness(const computation_msgs::RAD_VEGA_POPULATION _pop, std::string _title) {
    for (auto sol : _pop.solution) {
        std::cout << ros::Time::now() << ", " << powObjective(sol) << ", " << colObjective(sol) << std::endl;
    }
    // std::cout << "---------------END of pop fitness print for " << _title << "-------" << std::endl;
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
    int S = 20;
    float P = 10.0;
    SELECTED_POP_SIZE = S*(1 - (_computation_status.net_member.size()/P));
    // std::cout << "SELECTED_POP_SIZE: " << SELECTED_POP_SIZE << std::endl;
    // SELECTED_POP_SIZE = _computation_status.net_member.size();
    if (_pop.solution.size() < SELECTED_POP_SIZE) {SELECTED_POP_SIZE = _pop.solution.size();} 

    for (int i = 0; i < _pop.solution.size(); i++)
    {
        if (i < _pop.solution.size() / 2)
        {
            _fitness_f1.push_back(powObjective(_pop.solution.at(i)));
            _pop_f1.solution.push_back(_pop.solution.at(i));
        }
        else
        {
            _fitness_f2.push_back(colObjective(_pop.solution.at(i)));
            _pop_f2.solution.push_back(_pop.solution.at(i));
        }
    }
    
    for (int j = 0; j < SELECTED_POP_SIZE/2; j++) {
        float fit_min = 9999999.999;
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
        float fit_min = 9999999.999;
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
    // std::cout << NS + ": --------------msg.msg_source: " << msg.msg_source << std::endl;

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
        // std::cout << "------FROM RX-----" << std::endl;
        
    }
    addToInitSol(msg.ego_path);
    if(msg.computation_status.P.solution.size()>0 && msg.msg_source != NS) {
        _others_P.solution.insert(_others_P.solution.end(),
                                  msg.computation_status.P.solution.begin(),
                                  msg.computation_status.P.solution.end());
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
    _computation_status.CAV_t_available = 100;
}

void checkNetMembers(const ros::TimerEvent& event) {
    float NET_MEMBERS_MAX_DELAY = 10;
    bool change_in_net = false;
    CAV_total_flop = 0;
    computation_msgs::NET_MEMBER _updated_net_members;
    for (int i = 0; i < _computation_status.net_member.size(); i++) {
        if ((ros::Time::now() - _computation_status.net_member[i].header.stamp).toSec() > NET_MEMBERS_MAX_DELAY) {
            _computation_status.net_member.erase(_computation_status.net_member.begin() + i);
            change_in_net = true;
        } else {
            CAV_total_flop = CAV_total_flop + _computation_status.net_member[i].CAV_flop; 
        }
    }
    CAV_total_flop = CAV_total_flop + _computation_status.CAV_flop;
    if (change_in_net){
        ros::TimerEvent _tev;
        _tev.current_expected = ros::Time::now();
        _tev.current_real = ros::Time::now();
        // calcThisCAV_flp(_tev);
        // calcThisCAV_t_available(_tev);
        chooseLeader(_tev);
    } 
}
autoware_msgs::LaneArray downSampleLaneArr(const autoware_msgs::LaneArray msg) {
    autoware_msgs::LaneArray _filtered;
    autoware_msgs::Lane _tmp_lane;
    for (int i = 0; i < msg.lanes[0].waypoints.size(); i++) {
        if (i % 20 == 0){_tmp_lane.waypoints.push_back(msg.lanes[0].waypoints[i]);}
    }
    _filtered.id = msg.id;
    _filtered.lanes.push_back(_tmp_lane);
    return _filtered;
}
void egoPathCallback(const autoware_msgs::LaneArray msg)
{
    // std::cout << NS << ": ------waypoint.size(): " << msg.lanes[0].waypoints.size() << std::endl;

    addToInitSol(downSampleLaneArr(msg));
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
            if ((powObjective(_pop.solution[j]) < powObjective(_sol)
            && colObjective(_pop.solution[j]) < colObjective(_sol))
            || (powObjective(_pop.solution[j]) == powObjective(_sol)
            && colObjective(_pop.solution[j]) < colObjective(_sol))
            || (powObjective(_pop.solution[j]) < powObjective(_sol)
            && colObjective(_pop.solution[j]) == colObjective(_sol))) {
                _pareto_member_found = false;
                break;
            } else {_pareto_member_found = true;}
        }

        if (_pareto_member_found) {
            for (auto sol : _pareto_set.solution) {
                if(powObjective(sol) == powObjective(_sol)
                && colObjective(sol) == colObjective(_sol)) {
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
        //     std::cout << powObjective(_sol) << std::endl;
        // }
        // std::cout << "------------PARTEO SET, F2: ------------" << std::endl;
        // for (auto _sol: _pareto_set.solution) {
        //     std::cout << colObjective(_sol) << std::endl;
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
    int MAX_CROSS_COUNT = 1;
    
    // printPopFitness(_inpop, " CrossOver and Mutation input ");

    while (_inpop.solution.size() > 1 && cross_idx < MAX_CROSS_COUNT) {
        // std::cout << NS << ": ------------cross_idx: " << cross_idx << std::endl;
        _outpop.solution.clear();
        while (idx < MAX_MUT_COUNT) {
            computation_msgs::RAD_VEGA_SOLUTION _tmp_solution;
            NEXT_CANDIDATE:
            crossCandidIdx_1 = round((static_cast <float> (rand()) / static_cast <float> (RAND_MAX))*_inpop.solution.size());
            crossCandidIdx_2 = round((static_cast <float> (rand()) / static_cast <float> (RAND_MAX))*_inpop.solution.size());
            // std::cout << NS << ", solsize, crossCandidIdx_1, crossCandidIdx_2: " 
            // << _inpop.solution.size() << ", " << crossCandidIdx_1 << ", " << crossCandidIdx_2 << std::endl;
    
            int _so_tmp = _inpop.solution.size() - 1;
            crossCandidIdx_1 = std::min(crossCandidIdx_1,_so_tmp);
            crossCandidIdx_2 = std::min(crossCandidIdx_2,_so_tmp);
            float poc1 = powObjective(_inpop.solution[crossCandidIdx_1]); 
            float coc1 = colObjective(_inpop.solution[crossCandidIdx_1]); 
            float poc2 = powObjective(_inpop.solution[crossCandidIdx_2]); 
            float coc2 = colObjective(_inpop.solution[crossCandidIdx_2]);
            // std::cout<< NS << ", poc1: " << poc1 << ", poc2: " << poc2 << std::endl; 
            // std::cout<< NS << ", coc1: " << coc1 << ", coc2: " << coc2 << std::endl; 
            if (poc1 != poc2 || coc1 != coc2) {
                // std::cout << NS << ": ----------------------0" << std::endl;

                for (int k = 0; k < _inpop.solution[0].sol_set.size(); k++) {
                    autoware_msgs::LaneArray _lane_arr;
                    autoware_msgs::Lane _lane;
                    for (int i = 0; i < _inpop.solution[crossCandidIdx_1].sol_set.at(k).lanes.at(0).waypoints.size()/2;i++) {
                        autoware_msgs::Waypoint _wp;
                        _wp = _inpop.solution[crossCandidIdx_1].sol_set.at(k).lanes.at(0).waypoints[i];
                        _lane.waypoints.push_back(_wp);
                    }
                    for (int i = (_inpop.solution[crossCandidIdx_1].sol_set.at(k).lanes.at(0).waypoints.size()/2); i < _inpop.solution[crossCandidIdx_1].sol_set.at(k).lanes.at(0).waypoints.size();i++) {
                        autoware_msgs::Waypoint _wp;
                        _wp = _inpop.solution[crossCandidIdx_2].sol_set.at(k).lanes.at(0).waypoints[i];
                        _lane.waypoints.push_back(_wp);
                    }
                    _lane_arr.lanes.push_back(_lane);
                    _lane_arr.id = _inpop.solution[0].sol_set[k].id;
                    _tmp_solution.sol_set.push_back(_lane_arr);
                }
                int new_childs_try_count = 0;
                // std::cout << NS << ": ------------mut_idx: " << idx << " of total: " << MAX_MUT_COUNT << std::endl;
                TRY_NEW_CHILDS:
                // std::cout << NS << ": ------------new_childs_try_count: " << new_childs_try_count << std::endl;
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
                    }
                }
                if (_tmp_pareto.solution.size()>0 && child_inside) {
                    // std::cout << "----------------------1" << std::endl;
                    for (auto _sol : _tmp_pareto.solution) {
                        _outpop.solution.push_back(_sol);
                        idx++;
                    }
                } else if (new_childs_try_count != new_child_try_max) {
                    // std::cout << "----------------------new_childs_try_count: " << new_childs_try_count << std::endl;
                    
                    for (int k = 0; k < _tmp_solution.sol_set.size(); k++) {
                        float coef;
                        for (int i = 0; i < _tmp_solution.sol_set[k].lanes[0].waypoints.size(); i++) {
                            coef = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
                            // std::cout << NS << ": --------------coef: " << coef << std::endl;
                            if (coef < 0.5) {
                                _tmp_solution.sol_set[k].lanes[0].waypoints[i].twist.twist.linear.x = 21.0 - _tmp_solution.sol_set[k].lanes[0].waypoints[i].twist.twist.linear.x;
                            }
                        }
                    }
                    new_childs_try_count = new_childs_try_count + 1;
                    goto TRY_NEW_CHILDS;
                } else {
                    // std::cout << NS << ": NOT BETTER CHILD FOUND AT mut idx: " << idx << std::endl;
                    _outpop.solution.push_back(_inpop.solution[crossCandidIdx_1]);
                    _outpop.solution.push_back(_inpop.solution[crossCandidIdx_2]);
                    idx = idx + 2;
                }
            } else if (_inpop.solution.size() > 1)  {
                // std::cout << NS << ": ----------------------1" << std::endl;
                // std::cout << "----------------------4" << std::endl;
                goto NEXT_CANDIDATE;
            } else {
                // std::cout << NS << ": ----------------------2" << std::endl;
            }
        }
        cross_idx++;
        _inpop = _outpop;
        idx = 0;
    }
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
        float _f1 = powObjective(_pop.solution[i]);
        if(_f1 < 0.01) 
        {_result = _result + pow(1 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.05)
        {_result = _result + pow(0.77 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.1)
        {_result = _result + pow(0.67 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.15)
        {_result = _result + pow(0.6 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.21)
        {_result = _result + pow(0.54 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.26)
        {_result = _result + pow(0.48 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.31)
        {_result = _result + pow(0.43 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.37)
        {_result = _result + pow(0.39 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.42)
        {_result = _result + pow(0.35 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.47)
        {_result = _result + pow(0.31 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.52)
        {_result = _result + pow(0.27 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.58)
        {_result = _result + pow(0.24 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.63)
        {_result = _result + pow(0.2 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.68)
        {_result = _result + pow(0.17 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.73)
        {_result = _result + pow(0.14 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.79)
        {_result = _result + pow(0.11 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.84)
        {_result = _result + pow(0.08 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.89)
        {_result = _result + pow(0.05 - colObjective(_pop.solution[i]),2);}
        else if (_f1 < 0.94)
        {_result = _result + pow(0.02 - colObjective(_pop.solution[i]),2);}
        else {_result = _result + pow(- colObjective(_pop.solution[i]),2);}
    }
    return _result/_pop.solution.size();
}

void printWaypoints(const computation_msgs::RAD_VEGA_SOLUTION _sol) {
    for (auto sol : _sol.sol_set) {
        int idx = 0;
        std::cout << "From " << NS << " solution for vehcile v_" << sol.id << std::endl;
        std::cout << "From " << NS << " power & colision costs: " << powObjective(_sol) << ", " << colObjective(_sol) << std::endl;
        for (auto wp : sol.lanes[0].waypoints) {
            idx++;
            std::cout << idx << ", " << wp.twist.twist.linear.x << std::endl; 
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ZDT_solver_VEGA");
    ros::NodeHandle n;
    NS = argv[argc-1];

    n.param<int>("/ZDT_solver_VEGA/FLOP_CALC_PERIOD_SEC", FLOP_CALC_PERIOD_SEC, 2);
    n.param<int>("/ZDT_solver_VEGA/LOOP_REPS", LOOP_REPS, 10000000);
    n.param<int>("/ZDT_solver_VEGA/N_p", N_p, 1);
    n.param<float>("/ZDT_solver_VEGA/APPCO", APPCO, 0.00025);
    n.param<float>("/ZDT_solver_VEGA/c_vp_a", c_vp_a, 0.1);
    n.param<float>("/ZDT_solver_VEGA/c_vp_b", c_vp_b, 0.1);
    n.param<float>("/ZDT_solver_VEGA/c_ap_a", c_ap_a, 0.1);
    n.param<float>("/ZDT_solver_VEGA/c_ap_b", c_ap_b, 0.1);
    n.param<int>("/ZDT_solver_VEGA/t_min", t_min, 10.0);

    status_pub = n.advertise<computation_msgs::status>("/computation/status", 1);
    vega_stat_pub = n.advertise<computation_msgs::VEGA_stat>("/computation/VEGA_stat", 1);
    cooperation_status_pub = n.advertise<cooperative_msgs::status>("/computation/cooperation_status", 1);
    updated_lane_pub = n.advertise<autoware_msgs::LaneArray>("/computation/updated_lane_waypoints_array", 1);
    computation_sub = n.subscribe("/rx_com", 10000, rxCallback);
    ego_path_sub = n.subscribe("/lane_waypoints_array", 1, egoPathCallback);

    ros::Timer timer_FLOP_CALC = n.createTimer(ros::Duration(1.0), calcThisCAV_flp);
    ros::Timer timer_EAT_CALC = n.createTimer(ros::Duration(1.0), calcThisCAV_t_available);
    ros::Timer timer_leader_selection = n.createTimer(ros::Duration(10.0), chooseLeader);
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
    bool ch = false;
    bool ch_write = false;

    while (ros::ok())
    {
        int N_i = 0;
        if (_computation_status.CAV_flop != 0 && _computation_status.CAV_t_available != 0) {
            _vega_stat.N_i = N_p*_computation_status.CAV_flop*_computation_status.CAV_t_available*APPCO*t_min;
            // std::cout  << "---------_vega_stat.N_i: "<< _vega_stat.N_i << std::endl;
            N_i = _vega_stat.N_i;
            _vega_stat.N_i = N_i;
            
            while (_others_P.solution.size() > N_i) {
                ch = true;
                _others_P.solution.erase(_others_P.solution.begin());
            }

            if (_computation_status.net_member.size() > 0 
            && _init_sol.sol_set.size() == _computation_status.net_member.size() + 1) {
                // std::cout << "-------waypoint size: " << _init_sol.sol_set[0].lanes[0].waypoints.size() << std::endl;
                // std::cout << "-------_computation_status.net_member.size(): " << _computation_status.net_member.size() << std::endl;
                for (int i = 0; i < N_i - _others_P.solution.size(); i++) {
                    computation_msgs::RAD_VEGA_SOLUTION _tmp_sol;
                    for (auto laneArr : _init_sol.sol_set) {
                        // std::cout << NS << ", laneArr.id: "<< laneArr.id << std::endl;
                        // std::cout << "laneArr.lanes[0].waypoints: "<< laneArr.lanes[0].waypoints[0].pose.pose.position.x << std::endl;
                        autoware_msgs::Lane _tmp_lane;
                        autoware_msgs::LaneArray _tmp_laneArr;
                        for (int i = 0; i < laneArr.lanes[0].waypoints.size(); i++) {
                            autoware_msgs::Waypoint _wp;
                            _wp = laneArr.lanes[0].waypoints[i];
                            float rnd = 19.0*(static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
                            _wp.twist.twist.linear.x = rnd + 1.0;
                            
                            // std::cout << NS << ", _wp.twist.twist.linear.x: "<< _wp.twist.twist.linear.x << std::endl;
                            _tmp_lane.waypoints.push_back(_wp);
                        }
                        _tmp_laneArr.lanes.push_back(_tmp_lane);
                        _tmp_laneArr.id = laneArr.id;
                        _tmp_sol.sol_set.push_back(_tmp_laneArr);
                    }
                    P_i.solution.push_back(_tmp_sol);
                }
                P_i.solution.insert(P_i.solution.end(), _others_P.solution.begin(), _others_P.solution.end());
            }
            _vega_stat.NoOfAgents = _computation_status.net_member.size() + 1;
        }
        
        for (auto sol : _init_sol.sol_set) {
            // std::cout << NS << ", waypoints size: " << sol.lanes[0].waypoints.size() << std::endl;
        }
        
        int GA_IT = 20;
        // std::cout << "---------------------------------init_sol.sol_set.size(): " << _init_sol.sol_set.size() << std::endl;
        // std::cout << "---------------------------------_computation_status.net_member.size(): " << _computation_status.net_member.size() << std::endl;
        if (_others_P.solution.size() == 0) {GA_IT = 1;}
        if (P_i.solution.size() > 1) {
            // std::cout << "--------sol2,1:" << P_i.solution[2].sol_set[1].lanes[0].waypoints[0].pose.pose.position.x << std::endl;
            cout.setf(ios_base::fixed);
            clock_t rl_start = clock();
            computation_msgs::RAD_VEGA_POPULATION _tmp_pop;
            for (int k = 0; k < GA_IT; k++) {
                // std::cout << NS << "----------------------------GE: " << k << std::endl;
                
                _tmp_pop = P_i;
                // for (auto sol : P_i.solution) {
                //     for (auto laneArr : sol.sol_set) {
                //         if (laneArr.id == 0) {
                //             std::cout << "-----------NOT COOL------------" << std::endl;
                //             std::cout << "P_i.solution.size(): " << P_i.solution.size() << std::endl;
                //             std::cout << "P_i.solution.size(): " << sol.sol_set.size() << std::endl;
                //         }
                //     }
                // }
                int set0_size = P_i.solution[0].sol_set[0].lanes[0].waypoints.size();
                int set1_size = P_i.solution[0].sol_set[1].lanes[0].waypoints.size();
                for (auto sol : P_i.solution) {
                    if (sol.sol_set[0].lanes[0].waypoints.size() != set0_size
                    ||sol.sol_set[1].lanes[0].waypoints.size() != set1_size)
                    {std::cout << "--------NOT COOL----------" << std::endl;}
                }
                P_i = crossAndMate(P_i);
                // std::cout << NS << "--------AFTER crossAndMate P_i.solution.size():" << P_i.solution.size() << std::endl;
                
                if (P_i.solution.size() == 0) {
                    P_i = _tmp_pop;
                    if(_computation_status.IS_LEADER){std::cout << "-------WOOPS----------- " << std::endl;}
                    break;
                }
                _tmp_pop = P_i;
                // for (auto sol : P_i.solution) {
                //     for (auto laneArr : sol.sol_set) {
                //         if (laneArr.id == 0) {
                //             std::cout << "2:    -----------NOT COOL------------" << std::endl;
                //             std::cout << "2:    P_i.solution.size(): " << P_i.solution.size() << std::endl;
                //             std::cout << "2:    P_i.solution.size(): " << sol.sol_set.size() << std::endl;
                //         }
                //     }
                // }
                P_i = propSelection(P_i);
                // std::cout << NS << "--------AFTER propSelection P_i.solution.size():" << P_i.solution.size() << std::endl;
                if (P_i.solution.size() == 0) {
                    P_i = _tmp_pop;
                    if(_computation_status.IS_LEADER){std::cout << "-------WOOPS----------- " << std::endl;}
                    break;
                }
            }
            computation_msgs::RAD_VEGA_POPULATION _pareto_set;
            P_i.solution.insert(P_i.solution.end(), _others_P.solution.begin(), _others_P.solution.end());
            _pareto_set = findParetoSet(P_i, true);
            _computation_status.P = _pareto_set;
            computation_msgs::RAD_VEGA_SOLUTION _best_sol = findBestSolution(_pareto_set);
            clock_t rl_end = clock();
            double rl_time = difftime(rl_end, rl_start) / CLOCKS_PER_SEC;
            // std::cout << "From " << NS << "-------SIM TIME: " <<  ros::Time::now() << std::endl;
            // std::cout << "From " << NS << "-------EXE TIME: " <<  rl_time << std::endl;
            // std::cout << ros::Time::now() << ", " <<  rl_time << std::endl;
                // std::cout << "From " << NS << "-------Dg: " <<  Dg(_pareto_set) << std::endl;
            // std::cout <<  NS << ", " << _computation_status.CAV_flop << std::endl;
            // std::cout << ros::Time::now() << ", " << Dg(_pareto_set) << ", " << rl_time*_computation_status.CAV_flop << ", " << rl_time << std::endl;
            // std::cout << "sim time" 
            // << ", "  << "rl_time*_computation_status.CAV_flop" 
            // << ", " << "rl_time" 
            // << ", " << "_others_P.solution.size()" 
            // << ", " << "P_i.solution[0].sol_set.size()" 
            // std::cout << "init col/pow costs: " 
            // << colObjective(_init_sol)
            // << ", " << powObjective(_init_sol) 
            // << std::endl;
            float powObj_init_sol = powObjective(_init_sol);
            float colObj_init_sol = colObjective(_init_sol);
            float powObjErr = 0.0; 
            float colObjErr = 0.0; 
            for (auto sol : _pareto_set.solution) {
                powObjErr = powObjErr + (powObj_init_sol - powObjective(sol)); 
                colObjErr = colObjErr + (colObj_init_sol - colObjective(sol)); 
            }
            std::cout << ros::Time::now() 
            // << ", "  << rl_time*_computation_status.CAV_flop 
            // << ", " << rl_time 
            << ", " << N_i 
            << ", " << _others_P.solution.size() 
            << ", " << powObj_init_sol
            << ", " << colObj_init_sol
            // << ", " << P_i.solution[0].sol_set.size() 
            // << ", " << powObjErr/_pareto_set.solution.size()  
            // << ", " << colObjErr/_pareto_set.solution.size()  
            << std::endl;
            // if (ch) {std::cout << ros::Time::now() << ", " << N_i - _others_P.solution.size() << std::endl;}
            if (ch && !ch_write) {
                ch_write = true;
                // std::cout << ros::Time::now() << ", " << NS << std::endl;
            }
            bool PRINT_WAYPOINTS = false;
            if (ros::Time::now().toSec() > 300.0 && PRINT_WAYPOINTS) {
                printWaypoints(_init_sol);
                std::cout << "===============================================" << std::endl;
                printWaypoints(_pareto_set.solution[0]);
            }
            // if (NS == "v_1") {
            // std::cout << ros::Time::now() << ", " << rl_time*_computation_status.CAV_flop << std::endl;
            // }
            // std::cout << ros::Time::now() << ", " << NS << ", " << "N_sh: " <<  _others_P.solution.size() << std::endl;
            // std::cout << ros::Time::now() << ", " << NS << ", " <<  _others_P.solution.size() << std::endl;
                // return 0;
            // } else {
                // std::cout << NS << " is not ready" << std::endl;
            // }

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