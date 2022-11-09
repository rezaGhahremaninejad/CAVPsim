#include "ros/ros.h"
#include <signal.h>
#include <numeric>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <iostream>
#include <communication_msgs/BristolRx.h>
#include <communication_msgs/BristolTx.h>
#include <communication_msgs/ComMessage.h>
#include <computation_msgs/status.h>
#include <cav_vehicle_model_msgs/VehicleModelOutput.h>
#include <future>
#include <string>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
#include <pthread.h>

using namespace std;
using namespace communication_msgs;

namespace communication_model
{

    class itsg5_bristol : public nodelet::Nodelet
    {
    public:
        itsg5_bristol() : BAND_WIDTH(6), MESSAGE_RATE(0.1), PROC_DELAY_MEAN_SEC(0), PROC_DELAY_DEV_SEC(0), idx(0), NS("UNKNOWN") {}
        ~itsg5_bristol() {}

        // ros::CallbackQueue &callback_queue = getMTCallbackQueue();
        // vector<BristolTx> _Tx_Buffer;

    private:
        float BAND_WIDTH, MESSAGE_RATE, PROC_DELAY_DEV_SEC, PROC_DELAY_MEAN_SEC;
        string NS;
        int idx;
        ros::Subscriber tx_com_sub;
        ros::Publisher rx_com_pub;
        ros::SubscribeOptions ops;
        pthread_t a_thread;
        communication_msgs::ComMessage _tx_com;
        // float TRANSMISSION_RATE_SECS = 0.02;
        virtual void onInit()
        {
            ros::NodeHandle &private_nh = getPrivateNodeHandle();
            rx_com_pub = private_nh.advertise<communication_msgs::ComMessage>("/rx_com", 1);
            tx_com_sub = private_nh.subscribe("/tx_com", 1, &itsg5_bristol::comMessageCallback, this);
            private_nh.param<std::string>("NS", NS, "UNKNOWN");                             //wheelbase
        }

        // static void *msg_proc(void *threadid, float PROC_DELAY_MEAN_SEC,
        //                       float PROC_DELAY_DEV_SEC,
        //                       BristolTx msg,
        //                       float BAND_WIDTH,
        //                       float MESSAGE_RATE,
        //                       ros::Publisher v01_Rx_sim_pub)
        // {

        // struct mt_args
        // {
        //    float PROC_DELAY_MEAN_SEC;
        //    float PROC_DELAY_DEV_SEC;
        //    BristolTx msg;
        //    float BAND_WIDTH;
        //    float MESSAGE_RATE;
        //    ros::Publisher v01_Rx_sim_pub;   
        // };

        struct mt_args
        {
           float PROC_DELAY_MEAN_SEC;
           float PROC_DELAY_DEV_SEC;
           ComMessage msg;
           float BAND_WIDTH;
           float MESSAGE_RATE;
           ros::Publisher rx_com_pub;   
        };

        // static void *msg_proc(void *arguments)
        // {
        //     struct mt_args *args = (struct mt_args *)arguments;
        //     long tid;
        //     tid = (long)args->msg.header.seq;
        //     cout << "------------------- Thread ID, " << tid << endl;
        //     //pthread_exit(NULL);
        //     //for (int i = idx; i < _Tx_Buffer.size(); i++)
        //     // {
            
        //     communication_msgs::BristolRx _v01_Rx_sim;
        //     float PROC_DELAY_SEC = args->PROC_DELAY_MEAN_SEC +
        //                            (2 * args->PROC_DELAY_DEV_SEC * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX))) -
        //                            args->PROC_DELAY_DEV_SEC;
        //     //ros::Duration tr_rate(PROC_DELAY_SEC);
        //     ROS_INFO("----------------------------PROC_DELAY_SEC: %f", PROC_DELAY_SEC);

        //     _v01_Rx_sim.RxMAC = "4c:5e:0c:84:35:f6";
        //     //_v01_Rx_sim.header = msg.header;
        //     _v01_Rx_sim.SeqNum = args->msg.SeqNum;
        //     _v01_Rx_sim.GpsLon = args->msg.GpsLon;
        //     _v01_Rx_sim.GpsLat = args->msg.GpsLat;
        //     _v01_Rx_sim.CamLon = args->msg.CamLon;
        //     _v01_Rx_sim.CamLat = args->msg.CamLat;

        //     int s = sizeof(_v01_Rx_sim);
        //     float p = exp(-2 * s / (args->BAND_WIDTH * args->MESSAGE_RATE));
        //     ROS_INFO("----------SeqNum: %i", args->msg.SeqNum);
        //     //ROS_INFO("----------p: %f", p);
            
        //     if (p <= static_cast<float>(rand()) / static_cast<float>(RAND_MAX))
        //     {
        //         //ROS_INFO("----------AFTER SeqNum: %i", _Tx_Buffer[i].SeqNum);
        //         //_v01_Rx_sim.header.stamp = ros::Time::now();

        //         // com_status.vehicle_input_rx_timeStamp = vehicle_input_rx_timeStamp;
        //         // com_status.vehicle_input_tx_timeStamp = vehicle_input_tx.header.stamp;
        //         // com_status.total_delay_sec = com_status.vehicle_input_tx_timeStamp.toNSec() - com_status.vehicle_input_rx_timeStamp.toNSec();
        //         // com_status.missed_msgs = 1.0*vehicle_input_tx.header.seq - 1.0*seq;
        //         // seq = seq + 1;
        //         // _tx_ready = true;

        //         ros::Duration(PROC_DELAY_SEC).sleep();

        //         _v01_Rx_sim.header = args->msg.header;
        //         // _v01_Rx_sim.header.stamp = args->msg.header.stamp;
        //         _v01_Rx_sim.header.stamp = args->msg.header.stamp - ros::Duration(PROC_DELAY_SEC);
        //         // _v01_Rx_sim.header.stamp.secs = _v01_Rx_sim.header.stamp.secs + int(tr_rate.toSec());
        //         // _v01_Rx_sim.header.stamp.nsecs = _v01_Rx_sim.header.stamp.nsecs + int(tr_rate.toNSec());
        //         // _v01_Rx_sim.header.stamp.nsec += noise;
        //         // ROS_INFO("----------tr_rate sec : %i", int(ros::Time::now().toSec()));
        //         // ROS_INFO("----------NEW stamp ms : %f", float(ros::Time::now().toNSec())/1000000.0);
        //         _v01_Rx_sim.Timestamp = _v01_Rx_sim.header.stamp.toNSec() / 1000000;
        //         args->v01_Rx_sim_pub.publish(_v01_Rx_sim);
        //         // idx = idx + 1;
                
        //     }
        //     //}
        //     pthread_exit(NULL);
        //     return NULL;
        // }

        static void *comMsg_proc(void *arguments)
        {
            struct mt_args *args = (struct mt_args *)arguments;
            long tid;
            tid = (long)args->msg.header.seq;
            // cout << "------------------- Thread ID, " << tid << endl;
            //pthread_exit(NULL);
            //for (int i = idx; i < _Tx_Buffer.size(); i++)
            // {
            
            communication_msgs::ComMessage _rx_com;
            _rx_com = args->msg;
            float PROC_DELAY_SEC = args->PROC_DELAY_MEAN_SEC +
                                   (2 * args->PROC_DELAY_DEV_SEC * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX))) -
                                   args->PROC_DELAY_DEV_SEC;
            //ros::Duration tr_rate(PROC_DELAY_SEC);
            // ROS_INFO("----------------------------PROC_DELAY_SEC: %f", PROC_DELAY_SEC);

            
            int s = sizeof(_tx_com);
            float p = exp(-2 * s / (args->BAND_WIDTH * args->MESSAGE_RATE));
            // ROS_INFO("----------SeqNum: %i", args->msg.SeqNum);
            //ROS_INFO("----------p: %f", p);
            
            if (p <= static_cast<float>(rand()) / static_cast<float>(RAND_MAX))
            {
            
            
                // ROS_INFO("----------------------------OK TO SEND");
                //ROS_INFO("----------AFTER SeqNum: %i", _Tx_Buffer[i].SeqNum);
                //_v01_Rx_sim.header.stamp = ros::Time::now();

                // com_status.vehicle_input_rx_timeStamp = vehicle_input_rx_timeStamp;
                // com_status.vehicle_input_tx_timeStamp = vehicle_input_tx.header.stamp;
                // com_status.total_delay_sec = com_status.vehicle_input_tx_timeStamp.toNSec() - com_status.vehicle_input_rx_timeStamp.toNSec();
                // com_status.missed_msgs = 1.0*vehicle_input_tx.header.seq - 1.0*seq;
                // seq = seq + 1;
                // _tx_ready = true;

                ros::Duration(PROC_DELAY_SEC).sleep();

                // _rx_com.header = args->msg.header;
                // _rx_com.header.stamp = ros::Time::now();
                // _rx_com.header.stamp = args->msg.header.stamp - ros::Duration(PROC_DELAY_SEC);
                // _v01_Rx_sim.header.stamp = args->msg.header.stamp;
                // _v01_Rx_sim.header.stamp.secs = _v01_Rx_sim.header.stamp.secs + int(tr_rate.toSec());
                // _v01_Rx_sim.header.stamp.nsecs = _v01_Rx_sim.header.stamp.nsecs + int(tr_rate.toNSec());
                // _v01_Rx_sim.header.stamp.nsec += noise;
                // ROS_INFO("----------tr_rate sec : %i", int(ros::Time::now().toSec()));
                // ROS_INFO("----------NEW stamp ms : %f", float(ros::Time::now().toNSec())/1000000.0);
                // _rx_com.Timestamp = _rx_com.header.stamp.toNSec() / 1000000;
                args->rx_com_pub.publish(_rx_com);
                // idx = idx + 1;
                
            }
            //}
            pthread_exit(NULL);
            return NULL;
        }


        // void input_rxCallback(const communication_msgs::BristolTx msg)
        // {
            
        //     struct mt_args args;
        //     args.PROC_DELAY_MEAN_SEC = PROC_DELAY_MEAN_SEC;
        //     args.PROC_DELAY_DEV_SEC = PROC_DELAY_DEV_SEC;
        //     args.msg = msg;
        //     args.BAND_WIDTH = BAND_WIDTH;
        //     args.MESSAGE_RATE = MESSAGE_RATE;
        //     args.v01_Rx_sim_pub = v01_Rx_sim_pub;
        //     pthread_create(&a_thread, NULL, &msg_proc, (void *)&args);
        //     pthread_join(a_thread, NULL);
        // }


        void comMessageCallback(const communication_msgs::ComMessage msg)
        {
            struct mt_args args;
            args.PROC_DELAY_MEAN_SEC = PROC_DELAY_MEAN_SEC;
            args.PROC_DELAY_DEV_SEC = PROC_DELAY_DEV_SEC;
            args.msg.cav_vehicle_model_out = msg.cav_vehicle_model_out;
            args.msg.computation_status = msg.computation_status;
            args.msg.ego_path = msg.ego_path;
            args.msg.ego_status = msg.ego_status;
            args.msg.msg_source = msg.msg_source;
            args.msg.participants_status = msg.participants_status;
            args.msg.header.stamp = ros::Time::now();
            args.msg.msg_source = msg.msg_source;
            // args.msg.computation_status = msg.computation_status;
            args.BAND_WIDTH = BAND_WIDTH;
            args.MESSAGE_RATE = MESSAGE_RATE;
            args.rx_com_pub = rx_com_pub;
            pthread_create(&a_thread, NULL, &comMsg_proc, (void *)&args);
            pthread_join(a_thread, NULL);
        }

        // void compMesageCallback(const communication_msgs::ComMessage msg) {
        //     _tx_com.header = msg.header;
        //     _tx_com.computation_status = msg;
        //     tx_com_pub.publish(_tx_com);
        // }
    };

    PLUGINLIB_EXPORT_CLASS(communication_model::itsg5_bristol, nodelet::Nodelet)

} // namespace communication_model