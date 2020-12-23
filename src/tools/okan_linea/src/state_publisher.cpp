#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/console.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include <math.h>
#include <gps_common/GPSFix.h>
//#include <custom_msgs/gnssSample.h>

   double ox = 0;
   double oy = 0.000;
   double oz = 0;
   double ow = 1;

   float en_rx = 0;
   float en_ry = 0.000;
   float en_rz = 0;
   float en_rw = 1;

   double lax = 0;
   double lay = 0;
   double laz = 0;

   float pppx, pppy, pppz, ttlx,ttly, ttaz, ppox, ppoy, ppoz, ppow = 0;
   float rx, ry, rz = 0;
   float rw = 1;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
 {
   ox = msg->orientation.x;
   oy = msg->orientation.y;
   oz = msg->orientation.z;
   ow = msg->orientation.w;
/*
   lax = msg->linear_acceleration.x;
   lay = msg->linear_acceleration.y;
   laz = msg->linear_acceleration.z;
*/

 }

void encodersCallback(const nav_msgs::Odometry::ConstPtr& msg)
 {

   en_rx =  msg->pose.pose.orientation.x;
   en_ry =  msg->pose.pose.orientation.y;
   en_rz =  msg->pose.pose.orientation.z;
   en_rw =  msg->pose.pose.orientation.w;
/*
   rw = en_rw*rw - en_rx*rx - en_ry*ry - en_rz*rz;
   rx = en_rw*rx + en_rx*rw - en_ry*rz + en_rz*ry;
   ry = en_rw*ry + en_rx*rz + en_ry*rw - en_rz*rx;
   rz = en_rw*rz - en_rx*ry + en_ry*rx + en_rz*rw;
*/
 }

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
 {
   pppx = msg->pose.pose.position.x;
   pppy = msg->pose.pose.position.y;
   pppz = msg->pose.pose.position.z;

   ppox = msg->pose.pose.orientation.x;
   ppoy = msg->pose.pose.orientation.y;
   ppoz = msg->pose.pose.orientation.z;
   ppow = msg->pose.pose.orientation.w;

   ttlx = msg->twist.twist.linear.x;
   ttly = msg->twist.twist.linear.y;
   ttaz = msg->twist.twist.angular.z;
//the rotation from IMU and from EKF added to gether here

/*
   rw = ppow;
   rx = ppox;
   ry = ppoy;
   rz = ppoz;
*/
   rw = ow*ppow - ox*ppox - oy*ppoy - oz*ppoz;
   rx = ow*ppox + ox*ppow - oy*ppoz + oz*ppoy;
   ry = ow*ppoy + ox*ppoz + oy*ppow - oz*ppox;
   rz = ow*ppoz - ox*ppoy + oy*ppox + oz*ppow;

 }

 int main(int argc, char** argv){

   ros::init(argc, argv, "state_publisher");
   ros::NodeHandle n;
   ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odomFinal", 50);

     ros::NodeHandle m;
     ros::Subscriber imu_sub = m.subscribe("imu/data", 1000, imuCallback);

     ros::NodeHandle o;
     ros::Subscriber odom_sub = o.subscribe("odometry/filtered", 1000, odomCallback);

     ros::NodeHandle e;
     ros::Subscriber encoders_sub = e.subscribe("/diff_wheel_pos", 1000, encodersCallback);

   tf::TransformBroadcaster odom_broadcaster;
 
   ros::Time current_time, last_time;
   current_time = ros::Time::now();
   last_time = ros::Time::now();
 
   ros::Rate r(100);

   while(n.ok()){
     ros::spinOnce();               // check for incoming messages

     current_time = ros::Time::now();
 
     //compute odometry in a typical way given the velocities of the robot
     double dt = (current_time - last_time).toSec();

    //double temp = 2
    ROS_INFO("x: [%f], y: [%f]", pppx, pppy);
     
     //double delta_th = vth * dt;

     //th += delta_th;

     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(rz);

     //first, we'll publish the transform over tf
     geometry_msgs::TransformStamped odom_trans;
     odom_trans.header.stamp = current_time;
     odom_trans.header.frame_id = "odom";
     odom_trans.child_frame_id = "base_link";

     odom_trans.transform.translation.x = pppx;
     odom_trans.transform.translation.y = pppy;

     odom_trans.transform.translation.z = pppz;

     odom_trans.transform.rotation.x = rx;
     odom_trans.transform.rotation.y = ry;
     odom_trans.transform.rotation.z = rz;
     odom_trans.transform.rotation.w = rw;

     odom_broadcaster.sendTransform(odom_trans);
 
//next, we'll publish the odometry message over ROS
     nav_msgs::Odometry odom;
     odom.header.stamp = current_time;
     odom.header.frame_id = "odom";

     //set the position
     odom.pose.pose.position.x = pppx;
     odom.pose.pose.position.y = pppy;
     odom.pose.pose.position.z = pppz;
     //odom.pose.pose.orientation = odom_quat;
 
     odom.pose.pose.orientation.x = rx;
     odom.pose.pose.orientation.y = ry;
     odom.pose.pose.orientation.z = rz;
     odom.pose.pose.orientation.w = rw;

     //set the velocity
     odom.child_frame_id = "base_link";
     odom.twist.twist.linear.x = ttlx;
     odom.twist.twist.linear.y = ttly;
     odom.twist.twist.angular.z = ttaz;

     //publish the message
     odom_pub.publish(odom);
 
     last_time = current_time;
     
     r.sleep();
   }

   return 0;
 }
