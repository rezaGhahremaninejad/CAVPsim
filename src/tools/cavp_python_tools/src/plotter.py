#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt1

import rospy
from model_msgs.msg import VehicleModelOutput
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
machine_availabel_flops = []

vehicle1_seq = []
vehicle2_seq = []
vehicle1_navigation_cost = []
vehicle1_fuel_cost = []
vehicle1_collision_cost = []
vehicle2_navigation_cost = []
vehicle2_fuel_cost = []
vehicle2_collision_cost = []
solution_step = []

vehicle1Solution_vehicleStates_x1 = []
vehicle1Solution_vehicleStates_x2 = []
vehicle1Solution_vehicleStates_x3 = []
vehicle1Solution_vehicleStates_x4 = []

vehicle2Solution_vehicleStates_x1 = []
vehicle2Solution_vehicleStates_x2 = []
vehicle2Solution_vehicleStates_x3 = []
vehicle2Solution_vehicleStates_x4 = []

vehicle1Solution_VehicleControlSignals_u = []
vehicle2Solution_VehicleControlSignals_u = []

vehicle1Solution_VehicleControlSignals_w = []
vehicle2Solution_VehicleControlSignals_w = []

dataReceived = False
thisTime = rospy.Time()

def compStatus_plot_x(msg):
    global dataReceived
    global thisTime
    dataReceived = True
    execution_time.append(msg.execution_time)
    machine_availabel_flops.append(msg.machine_availabel_flops)
    time.append(1000*msg.header.stamp.secs + msg.header.stamp.nsecs/1000000)
    seq.append(msg.header.seq)
    thisTime = rospy.Time.now()

def solution_plot_x(msg):
    global dataReceived
    global thisTime
    dataReceived = True
    vehicle1_navigation_cost.append(msg.solution[-1].vehicle1_solution.navigation_cost)
    vehicle1_fuel_cost.append(msg.solution[-1].vehicle1_solution.fuel_cost)
    vehicle1_collision_cost.append(msg.solution[-1].vehicle1_solution.collision_risk)
    vehicle1Solution_vehicleStates_x1.append(msg.solution[-1].vehicle1_solution.vehicle_states.x_1)
    vehicle1Solution_vehicleStates_x2.append(msg.solution[-1].vehicle1_solution.vehicle_states.x_2)
    vehicle1Solution_vehicleStates_x3.append(msg.solution[-1].vehicle1_solution.vehicle_states.x_3)
    vehicle1Solution_vehicleStates_x4.append(msg.solution[-1].vehicle1_solution.vehicle_states.x_4)
    vehicle1Solution_VehicleControlSignals_u.append(msg.solution[-1].vehicle1_solution.vehicle_control_signals.u)
    vehicle1Solution_VehicleControlSignals_w.append(msg.solution[-1].vehicle1_solution.vehicle_control_signals.w)
    
    vehicle2_navigation_cost.append(msg.solution[-1].vehicle2_solution.navigation_cost)
    vehicle2_fuel_cost.append(msg.solution[-1].vehicle2_solution.fuel_cost)
    vehicle2_collision_cost.append(msg.solution[-1].vehicle2_solution.collision_risk)
    vehicle2Solution_vehicleStates_x1.append(msg.solution[-1].vehicle2_solution.vehicle_states.x_1)
    vehicle2Solution_vehicleStates_x2.append(msg.solution[-1].vehicle2_solution.vehicle_states.x_2)
    vehicle2Solution_vehicleStates_x3.append(msg.solution[-1].vehicle2_solution.vehicle_states.x_3)
    vehicle2Solution_vehicleStates_x4.append(msg.solution[-1].vehicle2_solution.vehicle_states.x_4)
    vehicle2Solution_VehicleControlSignals_u.append(msg.solution[-1].vehicle2_solution.vehicle_control_signals.u)
    vehicle2Solution_VehicleControlSignals_w.append(msg.solution[-1].vehicle2_solution.vehicle_control_signals.w)
    solution_step.append(msg.solution[-1].solution_step)
    time.append(1000*msg.solution[-1].vehicle1_solution.header.stamp.secs + msg.solution[-1].vehicle1_solution.header.stamp.nsecs/1000000)
    vehicle1_seq.append(msg.solution[-1].vehicle1_solution.header.seq)
    vehicle2_seq.append(msg.solution[-1].vehicle2_solution.header.seq)
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
            #label_str = 'average FLOPS: ' + str(sum(machine_availabel_flops)/len(machine_availabel_flops)) + ' FLOPS'
            #plt1.plot(time, machine_availabel_flops, label=label_str)

            #plt1.title('Navigation cost m^2')
            #label_str = 'vehicle1 navigation cost per solution step'
            #plt1.plot(solution_step, vehicle1_navigation_cost, label=label_str)

            #plt1.title('Navigation cost m^2')
            #label_str = 'vehicle2 navigation cost per solution step'
            #plt1.plot(solution_step, vehicle2_navigation_cost, label=label_str)
            
            #plt1.title('Fuel cost mg')
            #label_str = 'vehicle1 fuel cost per solution step, total: ' + str(sum(vehicle1_fuel_cost))
            #plt1.plot(solution_step, vehicle1_fuel_cost, label=label_str)

            #plt1.title('Fuel cost mg')
            #label_str = 'vehicle2 fuel cost per solution step, total: ' + str(sum(vehicle2_fuel_cost))
            #plt1.plot(solution_step, vehicle2_fuel_cost, label=label_str)

            #plt1.title('Collision risk 1/m^2')
            #label_str = 'vehicle1 collision risk per solution step'
            #plt1.plot(solution_step, vehicle1_collision_cost, label=label_str)

            #plt1.title('Collision risk 1/m^2')
            #label_str = 'vehicle2 collision risk per solution step'
            #plt1.plot(solution_step, vehicle2_collision_cost, label=label_str)

            #plt1.title('vehicle1 solution, position')
            #label_str = 'vehicle1 cartesian position, meter'
            #plt1.plot(vehicle1Solution_vehicleStates_x1, vehicle1Solution_vehicleStates_x2, label=label_str)

            #plt1.title('vehicle2 solution, position')
            #label_str = 'vehicle2 cartesian position, meter'
            #plt1.plot(vehicle2Solution_vehicleStates_x1, vehicle2Solution_vehicleStates_x2, label=label_str)

            #plt1.title('vehicle1 solution, speed')
            #label_str = 'vehicle1 speed m/s'
            #plt1.plot(time, vehicle1Solution_vehicleStates_x4, label=label_str)

            #plt1.title('vehicle2solution, speed')
            #label_str = 'vehicle2 speed m/s'
            #plt1.plot(time, vehicle2Solution_vehicleStates_x4, label=label_str)

            #plt1.title('vehicle1 solution, heading')
            #label_str = 'vehicle1 heading rad'
            #plt1.plot(time, vehicle1Solution_vehicleStates_x3, label=label_str)

            # plt1.title('vehicle2 solution, heading')
            # label_str = 'vehicle2 heading rad'
            # plt1.plot(time, vehicle2Solution_vehicleStates_x3, label=label_str)

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

