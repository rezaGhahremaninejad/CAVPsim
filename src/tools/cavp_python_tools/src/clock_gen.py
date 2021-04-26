#!/usr/bin/env python
import numpy as np
from scipy.io import loadmat
import rospy
from cav_vehicle_model_msgs.msg import VehicleModelOutput
from communication_msgs.msg import status, BristolTx, BristolRx
# from computation_msgs.msg import status
from solver_msgs.msg import finalSolutionArr
import rosbag
import csv

def clockCallback(msg):
    msg

def clock_gen():
    # mat_file = loadmat('path_to_mat.mat')
    global dataReceived
    global time
    rospy.Subscriber("/clock", Clock, clockCallback)
    # rospy.Subscriber("status", status, comStatus_plot_x)
    # rospy.Subscriber("comp1/status", status, compStatus_plot_x)
    # rospy.Subscriber("/solver_1/solution", finalSolutionArr, solution_plot_x)
    rospy.init_node('clock_gen', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():


        rate.sleep()

if __name__ == '__main__':
    try:
        clock_gen()
    except rospy.ROSInterruptException:
        pass

