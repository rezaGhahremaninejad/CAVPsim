#include "ros/ros.h"
#include "cav_vehicle_model_msgs/VehicleModelOutput.h"
#include "communication_msgs/ComMessage.h"
#include "computation_msgs/status.h"
#include "autoware_msgs/LaneArray.h"
#include "cooperative_msgs/status.h"
#include <cmath>

communication_msgs::ComMessage _tx_com;
float transmission_rate;
std::string NS;
int SIZE_OF_POSITIOIN_INFO_IN_BYTES = 4;

// void vehicleOutCallback(const cav_vehicle_model_msgs::VehicleModelOutput::ConstPtr &msg)
// {
//     // _tx_com.header.stamp = ros::Time::now();
//     _tx_com.cav_vehicle_model_out = *msg;
// }

void compStatusCallback(const computation_msgs::status::ConstPtr &msg)
{
    // _tx_com.header.stamp = ros::Time::now();
    _tx_com.computation_status = *msg;
}

void egoPathCAllBack(const autoware_msgs::LaneArray::ConstPtr & msg) {
    // _tx_com.header.stamp = ros::Time::now();
    _tx_com.ego_path = *msg;
}

// void coopStatusCallBack(const cooperative_msgs::status::ConstPtr & msg) {
//     // _tx_com.header.stamp = ros::Time::now();
//     _tx_com.ego_status = *msg;
// }

int getPacketSize(const communication_msgs::ComMessage msg) {
            int result = 0;
            for (auto lane : msg.ego_path.lanes) {
                for (auto waypoint : lane.waypoints) {
                    result = result + 4*SIZE_OF_POSITIOIN_INFO_IN_BYTES;
                }
            }
            for (auto sol : msg.computation_status.P.solution) {
                for (auto sol_set : sol.sol_set) {
                    for (auto lane : sol_set.lanes) {
                        for (auto waypoint : lane.waypoints) {
                            result = result + 4*SIZE_OF_POSITIOIN_INFO_IN_BYTES;
                        }
                    }
                }
            }
            return result
            + sizeof(msg.header)
            + sizeof(msg.msg_source)
            + sizeof(msg.ego_path.id)
            + sizeof(msg.computation_status.CAV_flop)
            + sizeof(msg.computation_status.CAV_t_available)
            + sizeof(msg.computation_status.header)
            + sizeof(msg.computation_status.id)
            + sizeof(msg.computation_status.IS_LEADER);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gate_node");
    NS = argv[argc-1];
    ros::NodeHandle n;
    n.param<float>("transmission_rate", transmission_rate, 0.1);                             //wheelbase
    ros::Publisher tx_pub = n.advertise<communication_msgs::ComMessage>("tx_com", 1);
    // ros::Subscriber vehicle_output_sub = n.subscribe("/" + NS + "/cav_vehicle_model/output", 1, vehicleOutCallback);
    ros::Subscriber computation_sub = n.subscribe("/" + NS + "/computation/status", 10000, compStatusCallback);
    ros::Subscriber ego_path_sub = n.subscribe("/" + NS + "/lane_waypoints_array", 1, egoPathCAllBack);
    // ros::Subscriber cooperation_status_sub = n.subscribe("/" + NS + "/computation/cooperation_status", 1, coopStatusCallBack);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        _tx_com.header.stamp = ros::Time::now();
        _tx_com.msg_source = NS;
        if (getPacketSize(_tx_com) > 1200 && _tx_com.computation_status.P.solution.size() > 0) {
            communication_msgs::ComMessage _tmp_com_msg;
            _tmp_com_msg = _tx_com;
            size_t MAX_SIZE = 1;
            _tmp_com_msg.computation_status.P.solution.erase(
                _tmp_com_msg.computation_status.P.solution.begin() + MAX_SIZE,
                _tmp_com_msg.computation_status.P.solution.end());
            // _tmp_com_msg.computation_status.P.solution.clear();
            // _tmp_com_msg.computation_status.P.solution.insert(
            //     _tmp_com_msg.computation_status.P.solution.begin(), 
            //     _tx_com.computation_status.P.solution.begin(), 
            //     _tx_com.computation_status.P.solution.begin() + MAX_SIZE);
            
            tx_pub.publish(_tmp_com_msg);
            _tx_com.computation_status.P.solution.erase(_tx_com.computation_status.P.solution.begin(), 
                _tx_com.computation_status.P.solution.begin() + MAX_SIZE);
            // if (_tx_com.computation_status.P.solution.size())
        } else {
            tx_pub.publish(_tx_com);
        }
        loop_rate.sleep();
    }
    return 0;
}