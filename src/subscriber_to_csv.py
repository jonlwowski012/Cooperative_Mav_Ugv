#!/usr/bin/env python

# Write information from subscriber to CSV

import rospy
from nav_msgs.msg import Odometry
import math
import numpy as np
import csv

header_flag = False

def callback21(data):
	ugv_orientation_x=data.pose.pose.orientation.x
	ugv_orientation_y=data.pose.pose.orientation.y
	ugv_orientation_z=data.pose.pose.orientation.z
	ugv_orientation_w=data.pose.pose.orientation.w
	e1=ugv_orientation_x
	e2=ugv_orientation_y
	e0=ugv_orientation_z
	e3=ugv_orientation_w
	
	time_stamp = data.header.stamp
	

	global header_flag
	
	with open('test.csv', 'a') as csv_file:
		writer = csv.DictWriter(csv_file, fieldnames=['Time Stamp', 'ugv_orientation_x', 'ugv_orientation_y', 'ugv_orientation_z', 'ugv_orientation_w'], lineterminator='\n')
		if(header_flag == False):
			writer.writeheader()
			header_flag = True
		writer.writerow({'Time Stamp': time_stamp, 'ugv_orientation_x': ugv_orientation_x, 'ugv_orientation_y': ugv_orientation_y , 'ugv_orientation_z': ugv_orientation_z , 'ugv_orientation_w': ugv_orientation_w})
		




def ugvnav():
	rospy.init_node('subscriber', anonymous=True)
	rospy.Subscriber("/p3dx/odom", Odometry, callback21)
	rospy.spin()




        
if __name__ == '__main__':
	ugvnav()
	try:
		a=1	
	except rospy.ROSInterruptException: pass
