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
        itsg5_bristol() : BAND_WIDTH(6), MESSAGE_RATE(0.2), PROC_DELAY_MEAN_SEC(0.12), PROC_DELAY_DEV_SEC(0.02), idx(0) {}
        ~itsg5_bristol() {}

        // ros::CallbackQueue &callback_queue = getMTCallbackQueue();
        // vector<BristolTx> _Tx_Buffer;

    private:
        float BAND_WIDTH, MESSAGE_RATE, PROC_DELAY_DEV_SEC, PROC_DELAY_MEAN_SEC;
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

            // ros::CallbackQueue callback_queue;

            // private_nh.setCallbackQueue(&callback_queue);
            // ros::NodeHandle &private_nh = getMTPrivateNodeHandle();

            // private_nh.getParam("CONTROLLER", CONTROLLER);

            // v01_Rx_sim_pub = private_nh.advertise<communication_msgs::BristolRx>("v01_Rx_sim", 1000);
            rx_com_pub = private_nh.advertise<communication_msgs::ComMessage>("/rx_com", 1000);
            // v00_Tx_sub = private_nh.subscribe("/v00_Tx_broadcast_0_215_215", 1000, &itsg5_bristol::input_rxCallback, this);
            tx_com_sub = private_nh.subscribe("/tx_com", 1000, &itsg5_bristol::comMessageCallback, this);
            // callback_queue.callOne(ros::WallDuration(1.0)); // can also be callAvailable()
            // callback_queue.callAvailable(ros::WallDuration()); // can also be callAvailable()
            // ops.template init<communication_msgs::BristolTx>("/v00_Tx_broadcast_0_215_215", 1000, &itsg5_bristol::input_rxCallback, this);
            // ops.transport_hints = ros::TransportHints();
            // ops.allow_concurrent_callbacks = true;
            // v00_Tx_sub = private_nh.subscribe(ops);
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
            args.msg = msg;
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