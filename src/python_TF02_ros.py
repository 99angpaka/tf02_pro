# -*- coding: utf-8 -*
# This script for Linux environment
import time
import serial
import rospy
from std_msgs.msg import Float32
from tf02_pro.msg import tf02pro

# lidar serial port setting
ser = serial.Serial("/dev/ttyUSB0", 115200)
# ROS node initalization
rospy.init_node('TF02_Lidar', anonymous=True)
pub = rospy.Publisher('TF02', tf02pro, queue_size=10)
rate = rospy.Rate(10)

def read_data():
    while True:
        counter = ser.in_waiting # count the number of bytes of the serial port
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()
            lidar_msg = tf02pro()
            lidar_msg.header.stamp = rospy.Time.now()
            lidar_msg.header.frame_id="laser"

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # this portion is for python3
                #print("Printing python3 portion")            
                distance = bytes_serial[2] + bytes_serial[3]*256
                print("Distance:"+ str(distance))
                ser.reset_input_buffer()

            if bytes_serial[0] == "Y" and bytes_serial[1] == "Y":
                distL = int(bytes_serial[2].encode("hex"), 16)
                distH = int(bytes_serial[3].encode("hex"), 16)
                distance = distL + distH*256
                lidar_msg.scan = distance
                pub.publish(lidar_msg)
                # print("Printing python2 portion")
                # print("Distance:"+ str(distance) + "\n")
                ser.reset_input_buffer()


if __name__ == "__main__":
    try:
        if ser.isOpen() == False:
            ser.open()
        read_data()
    except KeyboardInterrupt(): # ctrl + c in terminal.
        if ser != None:
            ser.close()
            print("program interrupted by the user")


