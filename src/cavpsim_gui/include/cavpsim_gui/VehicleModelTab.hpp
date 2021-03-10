namespace cavpsim_gui {
    
    // if (!ui.risk_desc_val->toPlainText().isEmpty()) {qnode.risk_status_msg.RISK_DESCRIPTION = ui.risk_desc_val->toPlainText().toStdString();} 
    void MainWindow::on_manual_accel_btn_clicked(bool check) {
        qnode.NEW_VEHCILE_CMD_MSG = true;
        qnode._vehicle_input_msg.vehicle_control_signals.u += 10;
    }

    void MainWindow::on_manual_brake_btn_clicked(bool check) {
        qnode.NEW_VEHCILE_CMD_MSG = true;
        qnode._vehicle_input_msg.vehicle_control_signals.u -= 10;
    }

    void MainWindow::on_manual_steering_right_btn_clicked(bool check) {
        // std::cout << "-----------------------------RIGHT" << std::endl; 
        qnode.NEW_VEHCILE_CMD_MSG = true;
        qnode._vehicle_input_msg.vehicle_control_signals.w += 0.02;
    }

    void MainWindow::on_manual_steering_left_btn_clicked(bool check) {
        qnode.NEW_VEHCILE_CMD_MSG = true;
        qnode._vehicle_input_msg.vehicle_control_signals.w -= 0.02;
    }
    
}