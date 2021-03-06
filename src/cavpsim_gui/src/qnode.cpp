/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/cavpsim_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cavpsim_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"cavpsim_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	vehicleA_ctrl_pub = n.advertise<VehicleModelInput>("vehicleA/cav_vehicle_model/input", 1000);
	vehicleB_ctrl_pub = n.advertise<VehicleModelInput>("vehicleB/cav_vehicle_model/input", 1000);
	// us_1 = n.subscribe("/USBoard/Sensor1", 10, &QNode::us_1_callBack, this);
	// n.getParam("adastec_hmi/BREAK_THRESHOLD_JOY_param",BREAK_THRESHOLD_JOY_param);

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"cavpsim_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	vehicleA_ctrl_pub = n.advertise<VehicleModelInput>("vehicleA/cav_vehicle_model/input", 1000);
	vehicleB_ctrl_pub = n.advertise<VehicleModelInput>("vehicleB/cav_vehicle_model/input", 1000);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(50);
	int count = 0;
	while ( ros::ok() ) {

		// if (NEW_VEHCILE_CMD_MSG)
		// {
		// 	NEW_VEHCILE_CMD_MSG = false;
		// 	vehicle_ctrl_pub.publish(_vehicle_input_msg);
		// 	//ROS_INFO(" ****************************** SENT");
		// }
		_vehicleA_input_msg.header.stamp = ros::Time::now();
		_vehicleB_input_msg.header.stamp = ros::Time::now();
		vehicleA_ctrl_pub.publish(_vehicleA_input_msg);
		vehicleB_ctrl_pub.publish(_vehicleB_input_msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

// void QNode::us_1_callBack(const sensor_msgs::Range msg)
// {
// 	us_1_measurement = msg.range;
// 	Q_EMIT us_1_EMIT();
// }

}  // namespace cavpsim_gui
