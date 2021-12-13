#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt1

import rospy
from cav_vehicle_model_msgs.msg import VehicleModelOutput
from communication_msgs.msg import status
# from computation_msgs.msg import status
from solver_msgs.msg import finalSolutionArr

x_1 = []
x_2 = []
x_3 = []
x_4 = []
x_5 = []
x_6 = []
x_7 = []

u = []
w = []
time = []
fuel_cost = []
seq = []

total_delay_sec = []
missed_messages = []

execution_time = []
CAV_flop = []

vehicleA_seq = []
vehicleB_seq = []
vehicleA_navigation_cost = []
vehicleA_fuel_cost = []
vehicleA_collision_cost = []
vehicleB_navigation_cost = []
vehicleB_fuel_cost = []
vehicleB_collision_cost = []
SOLVER_STEP = []

vehicleASolution_vehicleStates_x1 = []
vehicleASolution_vehicleStates_x2 = []
vehicleASolution_vehicleStates_x3 = []
vehicleASolution_vehicleStates_x4 = []

vehicleBSolution_vehicleStates_x1 = []
vehicleBSolution_vehicleStates_x2 = []
vehicleBSolution_vehicleStates_x3 = []
vehicleBSolution_vehicleStates_x4 = []

vehicleASolution_VehicleControlSignals_u = []
vehicleBSolution_VehicleControlSignals_u = []

vehicleASolution_VehicleControlSignals_w = []
vehicleBSolution_VehicleControlSignals_w = []

dataReceived = False
thisTime = rospy.Time()

def compStatus_plot_x(msg):
    global dataReceived
    global thisTime
    dataReceived = True
    execution_time.append(msg.execution_time)
    CAV_flop.append(msg.CAV_flop)
    time.append(1000*msg.header.stamp.secs + msg.header.stamp.nsecs/1000000)
    seq.append(msg.header.seq)
    thisTime = rospy.Time.now()

def solution_plot_x(msg):
    global dataReceived
    global thisTime
    dataReceived = True
    vehicleA_navigation_cost.append(msg.solution[-1].vehicleA_solution.navigation_cost)
    vehicleA_fuel_cost.append(msg.solution[-1].vehicleA_solution.fuel_cost)
    vehicleA_collision_cost.append(msg.solution[-1].vehicleA_solution.collision_risk)
    vehicleASolution_vehicleStates_x1.append(msg.solution[-1].vehicleA_solution.vehicle_states.x_1)
    vehicleASolution_vehicleStates_x2.append(msg.solution[-1].vehicleA_solution.vehicle_states.x_2)
    vehicleASolution_vehicleStates_x3.append(msg.solution[-1].vehicleA_solution.vehicle_states.x_3)
    vehicleASolution_vehicleStates_x4.append(msg.solution[-1].vehicleA_solution.vehicle_states.x_4)
    vehicleASolution_VehicleControlSignals_u.append(msg.solution[-1].vehicleA_solution.vehicle_control_signals.u)
    vehicleASolution_VehicleControlSignals_w.append(msg.solution[-1].vehicleA_solution.vehicle_control_signals.w)
    
    vehicleB_navigation_cost.append(msg.solution[-1].vehicleB_solution.navigation_cost)
    vehicleB_fuel_cost.append(msg.solution[-1].vehicleB_solution.fuel_cost)
    vehicleB_collision_cost.append(msg.solution[-1].vehicleB_solution.collision_risk)
    vehicleBSolution_vehicleStates_x1.append(msg.solution[-1].vehicleB_solution.vehicle_states.x_1)
    vehicleBSolution_vehicleStates_x2.append(msg.solution[-1].vehicleB_solution.vehicle_states.x_2)
    vehicleBSolution_vehicleStates_x3.append(msg.solution[-1].vehicleB_solution.vehicle_states.x_3)
    vehicleBSolution_vehicleStates_x4.append(msg.solution[-1].vehicleB_solution.vehicle_states.x_4)
    vehicleBSolution_VehicleControlSignals_u.append(msg.solution[-1].vehicleB_solution.vehicle_control_signals.u)
    vehicleBSolution_VehicleControlSignals_w.append(msg.solution[-1].vehicleB_solution.vehicle_control_signals.w)
    SOLVER_STEP.append(msg.solution[-1].SOLVER_STEP)
    time.append(1000*msg.solution[-1].vehicleA_solution.header.stamp.secs + msg.solution[-1].vehicleA_solution.header.stamp.nsecs/1000000)
    vehicleA_seq.append(msg.solution[-1].vehicleA_solution.header.seq)
    vehicleB_seq.append(msg.solution[-1].vehicleB_solution.header.seq)
    thisTime = rospy.Time.now()

def comStatus_plot_x(msg):
    global dataReceived
    global thisTime
    dataReceived = True
    total_delay_sec.append(msg.total_delay_sec)
    time.append(1000*msg.header.stamp.secs + msg.header.stamp.nsecs/1000000)
    seq.append(msg.header.seq)
    missed_messages.append(msg.missed_msgs)
    thisTime = rospy.Time.now()

def vehicleOutput_plot_x(msg):
    global dataReceived
    global thisTime
    dataReceived = True
    x_1.append(msg.vehicle_states.x_1)
    x_2.append(msg.vehicle_states.x_2)
    x_3.append(msg.vehicle_states.x_3)
    x_4.append(msg.vehicle_states.x_4)
    x_5.append(msg.vehicle_states.x_5)
    x_6.append(msg.vehicle_states.x_6)
    x_7.append(msg.vehicle_states.x_7)
    u.append(msg.vehicle_control_signals.u)
    w.append(msg.vehicle_control_signals.w)
    time.append(1000*msg.header.stamp.secs + msg.header.stamp.nsecs/1000000)
    fuel_cost.append(msg.fuel_cost)
    seq.append(msg.header.seq)
    thisTime = rospy.Time.now()

    #stamp = msg.header.stamp
    #time = stamp.secs + stamp.nsecs * 1e-9
    
    #plt1.pause(0.1)

def plotter():
    global dataReceived
    global time
    rospy.Subscriber("/output", VehicleModelOutput, vehicleOutput_plot_x)
    # rospy.Subscriber("status", status, comStatus_plot_x)
    # rospy.Subscriber("comp1/status", status, compStatus_plot_x)
    rospy.Subscriber("/solver_1/solution", finalSolutionArr, solution_plot_x)
    rospy.init_node('plotter', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        #rospy.loginfo("*************From main loop: [%d]", rospy.Time.now().to_sec() - thisTime.to_sec() + 2)
        if (dataReceived and rospy.Time.now().to_sec() > thisTime.to_sec() + 2):
            #rospy.loginfo("*************HERE")
            #plt1.ion()
            
            plt1.title('Vehicle position XY')
            plt1.plot(x_1, x_2,  label='Postion, m')
            time = [x - time[0] for x in time]

            #plt1.title('Vehicle Speed / mseconds')
            #plt1.plot(time, x_4, label='speed, m/s')

            # plt1.title('Requested toque / mseconds')
            # plt1.plot(time, u, label='Torque, N.m')

            # plt1.title('Requested steering rate / mseconds')
            # plt1.plot(time, w, label='Steering rate, rad/sec')

            #plt1.title('Vehicle orientation / mseconds')
            #plt1.plot(time, x_3, label='Orientation, rad')

            #plt1.title('Vehicle acceleration / mseconds')
            #plt1.plot(time, x_5, label='Acceleration, m/s^2')

            #plt1.title('Vehicle slip angle / mseconds')
            #plt1.plot(time, x_6, label='slip angle, rad')

            #plt1.title('Vehicle steer wheel angle / mseconds')
            #plt1.plot(time, x_7, label='Steering, rad')

            #plt1.title('Vehicle fuel consumption / mseconds')
            #plt1.plot(time, fuel_cost, label='fuel consumption, mgrams')

            # plt1.title('Communication delay / mseconds')
            # label_str = 'average delay: ' + str(sum(total_delay_sec)/len(total_delay_sec)) + ' nseconds'
            # plt1.plot(time, total_delay_sec, label=label_str)

            # plt1.title('Communication missed messages count')
            # lost_msg = 100*missed_messages[-1]/seq[-1]
            # if lost_msg > 100:
            #     lost_msg = 100
            # label_str = 'Total missed messages: ' + str(missed_messages[-1]) + ' %' + str(lost_msg) + ' lost'
            # plt1.plot(time, missed_messages, label=label_str)

            #plt1.title('Computation execution time / mseconds')
            #label_str = 'average execution time: ' + str(sum(execution_time)/len(execution_time)) + ' seconds'
            #plt1.plot(time, execution_time, label=label_str)

            #plt1.title('Computation resource FLOPS / mseconds')
            #label_str = 'average FLOPS: ' + str(sum(CAV_flop)/len(CAV_flop)) + ' FLOPS'
            #plt1.plot(time, CAV_flop, label=label_str)

            #plt1.title('Navigation cost m^2')
            #label_str = 'vehicleA navigation cost per solution step'
            #plt1.plot(SOLVER_STEP, vehicleA_navigation_cost, label=label_str)

            #plt1.title('Navigation cost m^2')
            #label_str = 'vehicleB navigation cost per solution step'
            #plt1.plot(SOLVER_STEP, vehicleB_navigation_cost, label=label_str)
            
            #plt1.title('Fuel cost mg')
            #label_str = 'vehicleA fuel cost per solution step, total: ' + str(sum(vehicleA_fuel_cost))
            #plt1.plot(SOLVER_STEP, vehicleA_fuel_cost, label=label_str)

            #plt1.title('Fuel cost mg')
            #label_str = 'vehicleB fuel cost per solution step, total: ' + str(sum(vehicleB_fuel_cost))
            #plt1.plot(SOLVER_STEP, vehicleB_fuel_cost, label=label_str)

            #plt1.title('Collision risk 1/m^2')
            #label_str = 'vehicleA collision risk per solution step'
            #plt1.plot(SOLVER_STEP, vehicleA_collision_cost, label=label_str)

            #plt1.title('Collision risk 1/m^2')
            #label_str = 'vehicleB collision risk per solution step'
            #plt1.plot(SOLVER_STEP, vehicleB_collision_cost, label=label_str)

            #plt1.title('vehicleA solution, position')
            #label_str = 'vehicleA cartesian position, meter'
            #plt1.plot(vehicleASolution_vehicleStates_x1, vehicleASolution_vehicleStates_x2, label=label_str)

            #plt1.title('vehicleB solution, position')
            #label_str = 'vehicleB cartesian position, meter'
            #plt1.plot(vehicleBSolution_vehicleStates_x1, vehicleBSolution_vehicleStates_x2, label=label_str)

            #plt1.title('vehicleA solution, speed')
            #label_str = 'vehicleA speed m/s'
            #plt1.plot(time, vehicleASolution_vehicleStates_x4, label=label_str)

            #plt1.title('vehicleBsolution, speed')
            #label_str = 'vehicleB speed m/s'
            #plt1.plot(time, vehicleBSolution_vehicleStates_x4, label=label_str)

            #plt1.title('vehicleA solution, heading')
            #label_str = 'vehicleA heading rad'
            #plt1.plot(time, vehicleASolution_vehicleStates_x3, label=label_str)

            # plt1.title('vehicleB solution, heading')
            # label_str = 'vehicleB heading rad'
            # plt1.plot(time, vehicleBSolution_vehicleStates_x3, label=label_str)

            plt1.axis("auto")
            plt1.legend()
            plt1.show()
            #plt1.draw()
            #plt1.pause(0.00000000001)
            dataReceived = False

        rate.sleep()

if __name__ == '__main__':
    try:
        plotter()
    except rospy.ROSInterruptException:
        pass

