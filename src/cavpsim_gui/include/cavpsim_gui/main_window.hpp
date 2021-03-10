/**
 * @file /include/cavpsim_gui/main_window.hpp
 *
 * @brief Qt based gui for cavpsim_gui.
 *
 * @date November 2010
 **/
#ifndef cavpsim_gui_MAIN_WINDOW_H
#define cavpsim_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace cavpsim_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
	void on_manual_accel_btn_clicked(bool check );
	void on_manual_brake_btn_clicked(bool check );
	void on_manual_steering_left_btn_clicked(bool check );
	void on_manual_steering_right_btn_clicked(bool check );

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace cavpsim_gui

#endif // cavpsim_gui_MAIN_WINDOW_H
