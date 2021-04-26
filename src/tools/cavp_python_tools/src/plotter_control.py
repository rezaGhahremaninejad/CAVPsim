#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt2

import rospy
from cav_vehicle_model_msgs.msg import VehicleModelOutput

x = []
y = []

def plot_x(msg):
    x.append(round(msg.vehicle_control_signals.u, 2))
    y.append(round(msg.header.stamp.to_sec()))

    plt2.plot(y, x,color='green')
    plt2.axis("auto")
    plt2.draw()
    plt2.pause(0.00000000001)



if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter_control")
    rospy.Subscriber("vehicleA/output", VehicleModelOutput, plot_x)
    plt2.ion()
    plt2.show()
    rospy.spin()
