#!/usr/bin/env python
import numpy as np
from scipy.io import loadmat
import rospy
from vehicle_model_msgs.msg import VehicleModelOutput
from communication_msgs.msg import status, BristolTx, BristolRx
# from computation_msgs.msg import status
from solver_msgs.msg import finalSolutionArr
import rosbag
import csv

Tx = True

def openCSV():
    msg = BristolRx()
    if Tx:
        msg = BristolTx()
    
    bag = rosbag.Bag('/home/reza/Desktop/v00_Tx_broadcast_0_215_215.bag', 'w')
    with open('/home/reza/reza_ws/src/tools/cavp_python_tools/resources/v00_Tx_broadcast_0_215_215.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                print row[0], row[1], row[2], row[3], row[4], row[5]
                line_count += 1
            else:
                # print type(row[0])
                if Tx:
                    msg.SeqNum = int(row[0])
                    msg.GpsLon = float(row[1])
                    msg.GpsLat = float(row[2])
                    msg.CamLon = float(row[3])
                    msg.CamLat = float(row[4])
                    msg.Timestamp = int(row[5])
                    msg.header.seq = int(row[0])
                else:
                    msg.RxMAC = row[0]
                    msg.SeqNum = int(row[1])
                    msg.GpsLon = float(row[2])
                    msg.GpsLat = float(row[3])
                    msg.CamLon = float(row[4])
                    msg.CamLat = float(row[5])
                    msg.Timestamp = int(row[6])
                    msg.header.seq = int(row[1])

                f = 1000
                msg.header.stamp.secs = int(msg.Timestamp/f)
                msg.header.stamp.nsecs = f*f*(msg.Timestamp - f*msg.header.stamp.secs)
                print msg.Timestamp
                t = rospy.Time()
                t.secs = msg.header.stamp.secs
                t.nsecs = msg.header.stamp.nsecs
                bag.write('v00_Tx_broadcast_0_215_215', msg, t)
                print row[0], row[1], row[2], row[3], row[4], row[5]
                # print(f'\t{row[0]} works in the {row[1]} department, and was born in {row[2]}.')
                line_count += 1
        # print(f'Processed {line_count} lines.')
    bag.close()

def mat_reader():
    # mat_file = loadmat('path_to_mat.mat')
    global dataReceived
    global time
    openCSV()
    # rospy.Subscriber("/output", VehicleModelOutput, vehicleOutput_plot_x)
    # rospy.Subscriber("status", status, comStatus_plot_x)
    # rospy.Subscriber("comp1/status", status, compStatus_plot_x)
    # rospy.Subscriber("/solver_1/solution", finalSolutionArr, solution_plot_x)
    rospy.init_node('mat_reader', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():


        rate.sleep()

if __name__ == '__main__':
    try:
        mat_reader()
    except rospy.ROSInterruptException:
        pass

