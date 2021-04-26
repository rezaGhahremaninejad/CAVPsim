/**
 * @file /include/cavpsim_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cavpsim_gui_QNODE_HPP_
#define cavpsim_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include "cav_vehicle_model_msgs/VehicleModelInput.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace cav_vehicle_model_msgs;

namespace cavpsim_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
	bool NEW_VEHCILE_CMD_MSG = false;
	// void us_1_callBack(const sensor_msgs::Range msg);
	VehicleModelInput _vehicleA_input_msg,_vehicleB_input_msg;

	// adastec_msgs::NodeCtrl redundant_node_ctrl_msg;


	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
	void us_1_EMIT();


private:
	int init_argc;
	char** init_argv;
	ros::Publisher vehicleA_ctrl_pub, vehicleB_ctrl_pub;
    QStringListModel logging_model;
	ros::Subscriber us_1; 

};

}  // namespace cavpsim_gui

#endif /* cavpsim_gui_QNODE_HPP_ */
