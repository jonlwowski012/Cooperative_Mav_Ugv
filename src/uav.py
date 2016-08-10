#!/usr/bin/env python

# Quadrotor Simulator
# Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com 
# Unmanned Systems Lab
# The University of Texas at San Antonio



############  Import Libraries   ##########################
import cv2
import cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped
import math
import numpy as np
from ete2 import Tree
import collections
import time
import numpy
import collections
import heapq

################  Global Variables    ############################

quad1pos = 0
quad1imu = 0
quad1sonar = 0
quad1pressure = 0
quad1magnetic = 0
quad1altimeter = 0
quad1imu_orientation_x=0
quad1imu_orientation_y=0
quad1imu_orientation_z=0
quad1imu_orientation_w=0
ugv_orientation_x=0
ugv_orientation_y=0
ugv_orientation_z=0
ugv_orientation_w=0
bearing=0
phi=0
theta=0
psi=0
nxc_red=0
nyc_red=0
nxc_blue=0
nyc_blue=0
nxc_green=0
nyc_green=0
nxc_pink=0
nyc_pink=0
nxc_orange=0
nyc_orange=0
ugv_yaw=0
max_right_red_x = 0
max_left_red_x = 0
max_right_blue_x = 0
max_left_blue_x = 0
max_right_pink_x = 0
max_left_pink_x = 0
max_right_green_x = 0
max_left_green_x = 0
max_right_red_y = 0
max_left_red_y = 0
max_right_blue_y = 0
max_left_blue_y = 0
max_right_pink_y = 0
max_left_pink_y = 0
max_right_green_y = 0
max_left_green_y = 0
ugv_right_obstacle1_left = 0    
ugv_right_obstacle1_right = 0    
ugv_left_obstacle1_left = 0   
ugv_left_obstacle1_right = 0   
obstacle1_left_obstacle2_left = 0  
obstacle1_left_obstacle2_right = 0  
obstacle1_right_obstacle2_left = 0  
obstacle1_right_obstacle2_right = 0 
obstacle2_left_obstacle3_left = 0  
obstacle2_left_obstacle3_right = 0  
obstacle2_right_obstacle3_left = 0  
obstacle2_right_obstacle3_right = 0
obstacle3_right_end = 0
obstacle3_left_end = 0
directions = 0
unrooted_tree = Tree("(((C2, C1)B3, (C4, C3)B4)A1,((C6, C5)B1, (C8, C7)B2)A2 );", format=1)
xc_red = 0
yc_red = 0
xc_blue = 0
yc_blue = 0
xc_green = 0
yc_green = 0
xc_pink = 0
yc_pink = 0
xc_orange = 0
yc_orange = 0
obstacle1_relative_distance= 0
obstacle2_relative_distance= 0
obstacle3_relative_distance= 0
ugv_relative_distance = 0
M1=0
M2=0
M3=0
ugv_front_x = 0
ugv_front_y = 0
bottom_blue_x = 0
bottom_blue_y = 0
bottom_green_x = 0
bottom_green_y = 0
bottom_pink_x = 0
bottom_pink_y = 0
ugv_to_blue = 0
ugv_to_green = 0
ugv_to_pink = 0
#############   Tree   Function     #######################

def Tree():
    return collections.defaultdict(Tree)

###########      Camera Functions   ###############################
class uav_image_red:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image",Image)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/uav/downward_cam/camera/image",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
		print e
    ### Start of Image Processing ######
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,100,100])
    upper_red = np.array([10,255,255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    mat=cv.GetMat(cv.fromarray(mask))
    moments=cv.Moments(mat)
    if((moments.m01>(-10000000000000)) and (moments.m01<10000000000000) and (moments.m00>(-1000000000000)) and (moments.m00<10000000000000) and (moments.m10>(-1000000000000)) and (moments.m10<10000000000000)):
		global yc_red
		global xc_red
		yc_red= moments.m01/moments.m00
		xc_red=moments.m10/moments.m00
		width, height = cv.GetSize(mat)
		global max_right_red_x
		global max_left_red_x
		global max_left_red_y
		global max_right_red_y
		global ugv_front_x
		global ugv_front_y
		contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
		cnt = contours[0]
		#print contours

		leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
		rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
		topmost = tuple(cnt[cnt[:,:,1].argmin()][0])
		max_left_red_x = leftmost[0]
		max_left_red_y = leftmost[1]
		max_right_red_x = rightmost[0]
		max_right_red_y = rightmost[1]
		ugv_front_x = topmost[0]
		ugv_front_y = topmost[1]

		# max_right_red_x = 0
		# for a in range(int(round(xc_red)), width, 3):
		#    for b in range (0, height, 3):
		#       if(mat[b,a] == 0):
		#          continue
		#       elif(a > max_right_red_x):
		#          max_right_red_x = a
		#          max_right_red_y = b
		# global max_left_red_x
		# global max_left_red_y
		# max_left_red_x = width
		# for a2 in range(int(round(xc_red)), 0, -3):
		#    for b2 in range (0, height, 3):
		#       if(mat[b2,a2] == 0):
		#          continue
		#       elif(a2 < max_left_red_x):
		#          max_left_red_x = a2
		#          max_left_red_y = b2

		global nxc_red
		global nyc_red
		nxc_red=xc_red-320
		nyc_red=yc_red-240
		focal=1690.0#1097.51
		q= nxc_red/focal
		global bearing
		bearing=math.atan(q)*((180.0)/(3.14159))
		# cv2.circle(cv_image,(int(max_right_red_x),int(max_right_red_y)),3,(0,0,255),-1)
		# cv2.circle(cv_image,(int(max_left_red_x),int(max_left_red_y)),3,(0,0,255),-1)
		# cv2.circle(cv_image,(int(max_right_blue_x),int(max_right_blue_y)),3,(0,0,255),-1)
		# cv2.circle(cv_image,(int(max_left_blue_x),int(max_left_blue_y)),3,(0,0,255),-1)
		# cv2.circle(cv_image,(int(max_right_green_x),int(max_right_green_y)),3,(0,0,255),-1)
		#    cv2.circle(cv_image,(int(max_left_green_x),int(max_left_green_y)),3,(0,0,255),-1)
		#    cv2.circle(cv_image,(int(max_right_pink_x),int(max_right_pink_y)),3,(0,0,255),-1)
		#    cv2.circle(cv_image,(int(max_left_pink_x),int(max_left_pink_y)),3,(0,0,255),-1)
		cv2.circle(cv_image,(int(xc_red),int(yc_red)),3,(0,0,255),-1)
		#    #cv2.imshow('Thresholded', mask)

		#    ### End of Image Processing ######
		cv2.imshow('UAV Image', cv_image)
		cv2.waitKey(1)
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e


class uav_image_red_2:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image",Image)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/uav/downward_cam/camera/image",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
		print e
    ### Start of Image Processing ######
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,100,100])
    upper_red = np.array([10,255,255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    mat=cv.GetMat(cv.fromarray(mask))
    moments=cv.Moments(mat)
    mean = mask.mean();
    if (mean>.01):
		global yc_red
		global xc_red
		yc_red= moments.m01/moments.m00
		xc_red=moments.m10/moments.m00
		width, height = cv.GetSize(mat)
		global nxc_red
		global nyc_red
		nxc_red=xc_red-(width/2)
		nyc_red=yc_red-(height/2)
		cv2.circle(cv_image,(int(xc_red),int(yc_red)),3,(0,0,255),-1)
		#cv2.imshow('Thresholded', mask)

		#### End of Image Processing ######
		cv2.imshow('UAV Image', cv_image)
		cv2.waitKey(1)
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e
			
class uav_image_orange:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image",Image)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/uav/downward_cam/camera/image",Image,self.callback)

  def callback(self,data):
	try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError, e:
		print e
    ### Start of Image Processing ######
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	lower_orange = np.array([10,0,0])
	upper_orange = np.array([25,255,255])
	mask = cv2.inRange(hsv, lower_orange, upper_orange)
	mat=cv.GetMat(cv.fromarray(mask))
	moments=cv.Moments(mat)
	if((moments.m01>(-10000000000000)) and (moments.m01<10000000000000) and (moments.m00>(-1000000000000)) and (moments.m00<10000000000000) and (moments.m10>(-1000000000000)) and (moments.m10<10000000000000)):
		global yc_orange
		global xc_orange
		yc_orange= moments.m01/moments.m00
		xc_orange=moments.m10/moments.m00
		width, height = cv.GetSize(mat)

		global nxc_orange
		global nyc_orange
		nxc_orange=xc_orange-320
		nyc_orange=yc_orange-240
		focal=1690.0#1097.51
		q= nxc_orange/focal
		global bearing
		bearing=math.atan(q)*((180.0)/(3.14159))
		#cv2.circle(cv_image,(int(xc_orange),int(yc_orange)),3,(0,0,255),-1)
		#cv2.imshow('Thresholded', mask)

		### End of Image Processing ######
		#cv2.imshow('UAV Image Orange', cv_image)
	cv2.waitKey(1)
	try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError, e:
		print e

class uav_image_blue:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image",Image)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/uav/downward_cam/camera/image",Image,self.callback)

  def callback(self,data):
	try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError, e:
		print e
		
	### Start of Image Processing ######
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	lower_blue = np.array([110,50,50])
	upper_blue = np.array([130,255,255])
	mask = cv2.inRange(hsv, lower_blue, upper_blue)
	mat=cv.GetMat(cv.fromarray(mask))
	moments=cv.Moments(mat)
	if((moments.m01>(-10000000000000)) and (moments.m01<10000000000000) and (moments.m00>(-1000000000000)) and (moments.m00<10000000000000) and (moments.m10>(-1000000000000)) and (moments.m10<10000000000000)):
			global yc_blue
			global xc_blue
			yc_blue= moments.m01/moments.m00
			xc_blue=moments.m10/moments.m00
			width, height = cv.GetSize(mat)
			global max_right_blue_x
			global max_left_blue_x
			global max_right_blue_y
			global max_left_blue_y
			global bottom_blue_x
			global bottom_blue_y

			contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
			cnt = contours[0]
			#print contours

			leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
			rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
			bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])
			max_left_blue_x = leftmost[0]
			max_left_blue_y = leftmost[1]
			max_right_blue_x = rightmost[0]
			max_right_blue_y = rightmost[1]
			bottom_blue_x = bottommost[0]
			bottom_blue_y = bottommost[1]

			# for a in range(int(round(xc_blue)), width, 3):
			#    for b in range (0, height, 3):
			#       if(mat[b,a] == 0):
			#          continue
			#       elif(a > max_right_blue_x):
			#          max_right_blue_x = a
			#          max_right_blue_y = b
			# global max_left_blue_x
			# global max_left_blue_y
			# max_left_blue_x = width
			# for a2 in range(int(round(xc_blue)), 0, -3):
			#    for b2 in range (0, height, 3):
			#       if(mat[b2,a2] == 0):
			#          continue
			#       elif(a2 < max_left_blue_x):
			#          max_left_blue_x = a2
			#          max_left_blue_y = b2

			global nxc_blue
			global nyc_blue
			nxc_blue=xc_blue-320
			nyc_blue=yc_blue-240
			focal=1690.0#1097.51
			q= nxc_blue/focal
			global bearing
			bearing=math.atan(q)*((180.0)/(3.14159))
			cv2.circle(cv_image,(int(max_right_blue_x),int(max_right_blue_y)),3,(0,0,255),-1)
			cv2.circle(cv_image,(int(max_left_blue_x),int(max_left_blue_y)),3,(0,0,255),-1)
			#cv2.imshow('Thresholded', mask)

	### End of Image Processing ######
	#cv2.imshow('UAV Image2', cv_image)
			cv2.waitKey(3)

			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			except CvBridgeError, e:
				print e
				
class uav_image_pink:
	
	def __init__(self):
		self.image_pub = rospy.Publisher("/image",Image)

		cv2.namedWindow("Image window", 1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/uav/downward_cam/camera/image",Image,self.callback)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e

	### Start of Image Processing ######

		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		lower_pink = np.array([143,0,0])
		upper_pink = np.array([162,255,255])
		mask = cv2.inRange(hsv, lower_pink, upper_pink)
		mat=cv.GetMat(cv.fromarray(mask))
		moments=cv.Moments(mat)
		if((moments.m01>(-10000000000000)) and (moments.m01<10000000000000) and (moments.m00>(-1000000000000)) and (moments.m00<10000000000000) and (moments.m10>(-1000000000000)) and (moments.m10<10000000000000)):
			global yc_pink
			global xc_pink
			yc_pink= moments.m01/moments.m00
			xc_pink= moments.m10/moments.m00
			width, height = cv.GetSize(mat)
			global max_right_pink_x
			global max_right_pink_y
			global max_left_pink_x
			global max_left_pink_y
			global bottom_pink_x
			global bottom_pink_y

			contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
			cnt = contours[0]
			#print contours

			leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
			rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
			bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])

			max_left_pink_x = leftmost[0]
			max_left_pink_y = leftmost[1]
			max_right_pink_x = rightmost[0]
			max_right_pink_y = rightmost[1]
			bottom_pink_x=bottommost[0]
			bottom_pink_y=bottommost[1]

			# max_right_pink_x = 0
			# for a in range(int(round(xc_pink)), width, 3):
			#    for b in range (0, height, 3):
			#       if(mat[b,a] == 0):
			#          continue
			#       elif(a > max_right_pink_x):
			#          max_right_pink_x = a
			#          max_right_pink_y = b
			# global max_left_pink_x
			# global max_left_pink_y
			# max_left_pink_x = width
			# for a2 in range(int(round(xc_pink)), 0, -3):
			#    for b2 in range (0, height, 3):
			#       if(mat[b2,a2] == 0):
			#          continue
			#       elif(a2 < max_left_pink_x):
			#          max_left_pink_x = a2
			#          max_left_pink_y = b2

			global nxc_pink
			global nyc_pink
			nxc_pink=xc_pink-320
			nyc_pink=yc_pink-240
			focal=1690.0#1097.51
			q= nxc_pink/focal
			global bearing
			bearing=math.atan(q)*((180.0)/(3.14159))
			# cv2.circle(cv_image,(int(max_right_pink_x),int(yc_pink)),3,(0,0,255),-1)
			# cv2.circle(cv_image,(int(max_left_pink_y),int(yc_pink)),3,(0,0,255),-1)
			#cv2.imshow('Thresholded', mask)

			### End of Image Processing ######
			#cv2.imshow('UAV Image', cv_image)
			cv2.waitKey(1)

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e

class uav_image_green:

	def __init__(self):
		self.image_pub = rospy.Publisher("/image",Image)
		cv2.namedWindow("Image window", 1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/uav/downward_cam/camera/image",Image,self.callback)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e
    ### Start of Image Processing ######

		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		lower_green = np.array([50, 50, 50])
		upper_green = np.array([70, 255, 255])
		mask = cv2.inRange(hsv, lower_green, upper_green)
		mat=cv.GetMat(cv.fromarray(mask))
		moments=cv.Moments(mat)
		if((moments.m01>(-10000000000000)) and (moments.m01<10000000000000) and (moments.m00>(-1000000000000)) and (moments.m00<10000000000000) and (moments.m10>(-1000000000000)) and (moments.m10<10000000000000)):
			global yc_green
			global xc_green
			yc_green= moments.m01/moments.m00
			xc_green=moments.m10/moments.m00
			width, height = cv.GetSize(mat)
			global max_right_green_x
			global max_right_green_y
			global max_left_green_x
			global max_left_green_y
			global bottom_green_y
			global bottom_green_x

			contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
			cnt = contours[0]
			#print contours

			leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
			rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
			bottommost = tuple(cnt[cnt[:,:,1].argmax()][0])

			max_left_green_x = leftmost[0]
			max_left_green_y = leftmost[1]
			max_right_green_x = rightmost[0]
			max_right_green_y = rightmost[1]
			bottom_green_x = bottommost[0]
			bottom_green_y = bottommost[1]

			# max_right_green_x = 0
			# for a in range(int(round(xc_green)), width, 3):
			#    for b in range (0, height, 3):
			#       if(mat[b,a] == 0):
			#          continue
			#       elif(a > max_right_green_x):
			#          max_right_green_x = a
			#          max_right_green_y = b
			# global max_left_green_x
			# global max_left_green_y
			# max_left_green_x = width
			# for a2 in range(int(round(xc_green)), 0, -3):
			#    for b2 in range (0, height, 3):
			#       if(mat[b2,a2] == 0):
			#          continue
			#       elif(a2 < max_left_green_x):
			#          max_left_green_x = a2
			#          max_left_green_y = b2

			global nxc_green
			global nyc_green
			nxc_green=xc_green-320
			nyc_green=yc_green-240
		focal=1690.0#1097.51
		q= nxc_green/focal
		global bearing
		bearing=math.atan(q)*((180.0)/(3.14159))
			# cv2.circle(cv_image,(int(max_right_green_x),int(yc_green)),3,(0,0,255),-1)
			# cv2.circle(cv_image,(int(max_left_green_y),int(yc_green)),3,(0,0,255),-1)
			#cv2.imshow('Thresholded', mask)

		### End of Image Processing ######
		#cv2.imshow('UAV Image', cv_image)
		cv2.waitKey(1)

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e  
    

##################    Topic Callbacks    #############################

def callback5(data):
    global quad1imu
    global quad1imu_orientation_x
    global quad1imu_orientation_y
    global quad1imu_orientation_z
    global quad1imu_orientation_w
    quad1imu = data
    quad1imu_orientation_x=data.orientation.x
    quad1imu_orientation_y=data.orientation.y
    quad1imu_orientation_z=data.orientation.z
    quad1imu_orientation_w=data.orientation.w
    
def callback12(data):
    global quad1sonar
    quad1sonar = data.range

def callback16(data):
    global quad1pressure
    quad1pressure = data

def callback20(data):
    global quad1magnetic
    quad1magnetic = data

def callback21(data):
    global ugv_orientation_x
    global ugv_orientation_y
    global ugv_orientation_z
    global ugv_orientation_w
    ugv_orientation_x=data.pose.pose.orientation.x
    ugv_orientation_y=data.pose.pose.orientation.y
    ugv_orientation_z=data.pose.pose.orientation.z
    ugv_orientation_w=data.pose.pose.orientation.w
    e1=ugv_orientation_x
    e2=ugv_orientation_y
    e0=ugv_orientation_z
    e3=ugv_orientation_w
    #Heading
    global ugv_yaw
    ugv_yaw = math.atan2(2*(e0*e3+e1*e2),(e0**2+e1**2-e2**2-e3**2))
    ugv_yaw=ugv_yaw*180/math.pi

####################     Topic Subscribers   ####################
def imuquad1():
    rospy.Subscriber("/uav/raw_imu", Imu, callback5)  

def sonarquad1():
    rospy.Subscriber("/uav/sonar_height", Range, callback12) 

def pressurequad1():
    rospy.Subscriber("/uav/pressure_height", PointStamped, callback16)

def magneticquad1():
    rospy.Subscriber("/uav/magnetic", Vector3Stamped, callback20)

def ugvnav():
    rospy.Subscriber("/p3dx/odom", Odometry, callback21)

###############   Convert Quaternion to Euler  #########################
def QuaterniontoEuler():
    e1=quad1imu_orientation_x
    e2=quad1imu_orientation_y
    e0=quad1imu_orientation_z
    e3=quad1imu_orientation_w
    
    #Pitch
    theta= math.atan2(2*(e0*e1+e2*e3),(e0**2+e3**2-e1**2-e2**2))
    #Roll
    phi= math.asin(2*(e0*e2-e1*e3))
    #Heading
    psi = math.atan2(2*(e0*e3+e1*e2),(e0**2+e1**2-e2**2-e3**2))

    global phi
    phi=phi*180/math.pi
    global theta
    theta=theta*180/math.pi
    global psi
    psi=psi*180/math.pi

def takeoff(height):
    pub1 = rospy.Publisher('/uav/cmd_vel', Twist)
    

    rospy.init_node('takeoff', anonymous=True)
    r = rospy.Rate(10) # 10hz
    command = Twist()
    command.linear.x = 0.00
    command.linear.y = 0.00
    command.linear.z = 1.00
    command.angular.x = 0.00
    command.angular.y = 0.00
    command.angular.z = 0.00
    hover = Twist()
    hover.linear.x = 0.00
    hover.linear.y = 0.00
    hover.linear.z = 0.00
    hover.angular.x = 0.00
    hover.angular.y = 0.00
    hover.angular.z = 0.00
    x = 0
    while (quad1sonar < height):
        pub1.publish(command)      
    pub1.publish(hover)

def hover(time):

    pub1 = rospy.Publisher('/uav/cmd_vel', Twist)
    
    

    rospy.init_node('takeoff', anonymous=True)
    r = rospy.Rate(10) # 10hz
    command = Twist()
    command.linear.x = 0.00
    command.linear.y = 0.00
    command.linear.z = 0.00
    command.angular.x = 0.00
    command.angular.y = 0.00
    command.angular.z = 0.00
    x = 0
    for x in range(0, time):
	
        #str = "hello world %s"%rospy.get_time()
        #rospy.loginfo(str)
        pub1.publish(command)
    QuaterniontoEuler()
       
def land(time):
    pub1 = rospy.Publisher('/uav/cmd_vel', Twist)
   
 
    
    rospy.init_node('takeoff', anonymous=True)
    r = rospy.Rate(10) # 10hz
    command = Twist()
    command.linear.x = 0.00
    command.linear.y = 0.00
    command.linear.z = -1.00
    command.angular.x = 0.00
    command.angular.y = 0.00
    command.angular.z = 0.00
    x = 0
    for x in range(0, time):
        pub1.publish(command)
  
def set_velocity_uav(lx1, ly1, lz1, ax1, ay1, az1):   
    pub1 = rospy.Publisher('/uav/cmd_vel', Twist)


   
    rospy.init_node('takeoff', anonymous=True)
    r = rospy.Rate(10) # 10hz
    command1 = Twist()
    command1.linear.x = lx1
    command1.linear.y = ly1
    command1.linear.z = lz1
    command1.angular.x = ax1
    command1.angular.y = ay1
    command1.angular.z = az1
    hover = Twist()
    hover.linear.x = 0.0
    hover.linear.y = 0.0
    hover.linear.z = 0.0
    hover.angular.x = 0.0
    hover.angular.y = 0.0
    hover.angular.z = 0.0
    pub1.publish(command1)
    QuaterniontoEuler()

def set_velocity_ugv(time, lx1, ly1, lz1, ax1, ay1, az1):
    pub1 = rospy.Publisher('/p3dx/cmd_vel', Twist)
    rospy.init_node('takeoff', anonymous=True)
    r = rospy.Rate(10) # 10hz
    command1 = Twist()
    command1.linear.x = lx1
    command1.linear.y = ly1
    command1.linear.z = lz1
    command1.angular.x = ax1
    command1.angular.y = ay1
    command1.angular.z = az1
    hover = Twist()
    hover.linear.x = 0.0
    hover.linear.y = 0.0
    hover.linear.z = 0.0
    hover.angular.x = 0.0
    hover.angular.y = 0.0
    hover.angular.z = 0.0
    q=0
    for q in range(0, time):
	pub1.publish(command1)
	QuaterniontoEuler()
  
    pub1.publish(hover)
		
def track_ugv(time): 
    D_yaw = 1   
    Derivator_yaw = 0
    error_yaw = 0;
    Kd_yaw = 1
    D_pitch = 1   
    Derivator_pitch = 0
    error_pitch = 0;
    Kd_pitch = 1
    Derivator_roll = 0
    error_roll = 0;
    Kd_roll = 1
    for q in range(0, time):
	fill_tree()
	calculate_distances()

	##############   dijkstra   #####################       
	graph = Graph()
	graph.add_vertex('0')
	graph.add_vertex('1') 
	graph.add_vertex('2') 
	graph.add_vertex('3') 
	graph.add_vertex('4') 
	graph.add_vertex('5') 
	graph.add_vertex('6')
	graph.add_vertex('7')

	graph.add_edge('0', '1', ugv_right_obstacle1_left)
	graph.add_edge('0', '2', ugv_left_obstacle1_right)
	graph.add_edge('1', '3', obstacle1_left_obstacle2_left)
	graph.add_edge('1', '4', obstacle1_left_obstacle2_right)
	graph.add_edge('2', '3', obstacle1_right_obstacle2_left)
	graph.add_edge('2', '4', obstacle1_right_obstacle2_right)
	graph.add_edge('3', '5', obstacle2_left_obstacle3_left)
	graph.add_edge('3', '6', obstacle2_left_obstacle3_right)
	graph.add_edge('4', '5', obstacle2_right_obstacle3_left)
	graph.add_edge('4', '6', obstacle2_right_obstacle3_right)
	graph.add_edge('5', '7', obstacle3_left_end)
	graph.add_edge('6', '7', obstacle3_right_end)
	for v in graph:
		for w in v.get_connections():
			vid = v.get_id()
			wid = w.get_id()
	dijkstra(graph, graph.get_vertex('0'), graph.get_vertex('7'))
	target = graph.get_vertex('7')
	path = [target.get_id()]
	shortest(target, path)
	#print 'The shortest path : %s' %(path[::-1])
	send_dijkstra(path[::-1])

	###### Shortest Path    ###################
	#calculate_smallest()
	#send()


	constant = quad1sonar
    ############# YAW PID   #####################
        diff_yaw = angle_difference(psi,ugv_yaw)
        sign_yaw = turn_direction(psi,ugv_yaw)
        P_yaw = sign_yaw*diff_yaw*.1
        error_yaw = diff_yaw
        D_yaw = Kd_yaw * (error_yaw - Derivator_yaw)
        PD_yaw = P_yaw + D_yaw
        Derivator_yaw= error_yaw
        Kd_yaw = D_yaw
   ############## Pitch PID   #####################
        P_pitch=((-.0005*constant)*(nyc_red-120))
    	error_pitch = nyc_red
        D_pitch = Kd_pitch * (error_pitch - Derivator_pitch)
        PD_pitch = P_pitch + D_pitch
        Derivator_pitch = error_pitch
        Kd_pitch = D_pitch
  ############# Roll PID      ##################### 
        P_roll=((-.0005*constant)*nxc_red)
    	error_roll = nxc_red
        D_roll = Kd_roll * (error_roll - Derivator_roll)
        PD_roll = P_roll + D_roll
        Derivator_roll = error_roll
        Kd_roll = D_roll
	set_velocity_uav(PD_pitch, PD_roll, 0, 0, 0, PD_yaw)
    #print unrooted_tree.get_ascii(attributes=["name", "distance"], show_internal=True)	
    
def track_ugv_2(time): 
    D_yaw = 1   
    Derivator_yaw = 0
    error_yaw = 0;
    Kd_yaw = 1
    D_pitch = 1   
    Derivator_pitch = 0
    error_pitch = 0;
    Kd_pitch = 1
    Derivator_roll = 0
    error_roll = 0;
    Kd_roll = 1
    for q in range(0, time):
		constant = quad1sonar
		############# YAW PID   #####################
		diff_yaw = angle_difference(psi,ugv_yaw)
		sign_yaw = turn_direction(psi,ugv_yaw)
		P_yaw = sign_yaw*diff_yaw*.1
		error_yaw = diff_yaw
		D_yaw = Kd_yaw * (error_yaw - Derivator_yaw)
		PD_yaw = P_yaw + D_yaw
		Derivator_yaw= error_yaw
		Kd_yaw = D_yaw
		############## Pitch PID   #####################
		P_pitch=((-.0005*constant)*(nyc_red))
		error_pitch = nyc_red
		D_pitch = Kd_pitch * (error_pitch - Derivator_pitch)
		PD_pitch = P_pitch + D_pitch
		Derivator_pitch = error_pitch
		Kd_pitch = D_pitch
		############# Roll PID      ##################### 
		P_roll=((-.0005*constant)*nxc_red)
		error_roll = nxc_red
		D_roll = Kd_roll * (error_roll - Derivator_roll)
		PD_roll = P_roll + D_roll
		Derivator_roll = error_roll
		Kd_roll = D_roll
		set_velocity_uav(PD_pitch, PD_roll, 0, 0, 0, PD_yaw)
    #print unrooted_tree.get_ascii(attributes=["name", "distance"], show_internal=True)			           	          
def turn_direction(angle_init, ugv_angle):
   if(angle_init >= 0):
      angle_back = angle_init-180
   else:
      angle_back = angle_init+180
   if(angle_init >=0 and ugv_angle <= angle_init and ugv_angle >= angle_back):
      turn_sign = 1
   elif(angle_init >=0 and ((ugv_angle >= angle_init and ugv_angle <=180) or (ugv_angle <= angle_back))):
      turn_sign = -1
   elif(angle_init < 0 and ((ugv_angle <= angle_init and ugv_angle >=-180) or (ugv_angle >= angle_back))):
      turn_sign = 1
   elif(angle_init < 0 and ugv_angle > angle_init and ugv_angle < angle_back):
      turn_sign = -1
   return turn_sign

def angle_difference(uav_angle, ugv_angle):
   if(uav_angle >=0 and ugv_angle >=0):
      diff=abs(uav_angle - ugv_angle)
   elif(uav_angle >=0 and ugv_angle <0):
      diff = abs(360-uav_angle-abs(ugv_angle))
   elif(uav_angle <0 and ugv_angle >=0):
      diff = abs( abs(uav_angle) + ugv_angle)
   elif(uav_angle <0 and ugv_angle < 0):
      diff = abs(uav_angle - ugv_angle)
   if(diff > 180):
      diff = abs(360-diff)
   return diff

def track_yaw(time):
    D = 1   
    Derivator = 0
    error = 0;
    Kd = 1
    for q in range(0, time):
        diff = angle_difference(psi,ugv_yaw)
        sign = turn_direction(psi,ugv_yaw)
        P = sign*diff*.1
        error = diff
        D = Kd * (error - Derivator)
        PD = P + D
	set_velocity_uav(0, 0, 0, 0, 0, PD)
        Derivator= error
        Kd = D     

def calculate_distances():
   global ugv_right_obstacle1_left    
   global ugv_right_obstacle1_right    
   global ugv_left_obstacle1_left   
   global ugv_left_obstacle1_right   
   global obstacle1_left_obstacle2_left  
   global obstacle1_left_obstacle2_right  
   global obstacle1_right_obstacle2_left  
   global obstacle1_right_obstacle2_right 
   global obstacle2_left_obstacle3_left  
   global obstacle2_left_obstacle3_right  
   global obstacle2_right_obstacle3_left  
   global obstacle2_right_obstacle3_right
   global obstacle3_right_end 
   global obstacle3_left_end 
   global obstacle1_relative_distance
   global obstacle2_relative_distance
   global obstacle3_relative_distance
   global ugv_relative_distance
   global ugv_to_blue
   global ugv_to_green
   global ugv_to_pink

   ugv_right_obstacle1_left = math.hypot((max_left_red_x - max_left_blue_x),( max_right_red_y - max_left_blue_y) )

   ugv_right_obstacle1_right = math.hypot(max_right_red_x - max_right_blue_x, max_right_red_y - max_right_blue_y) 
  
   ugv_left_obstacle1_left = math.hypot(max_left_red_x - max_left_blue_x, max_left_red_y - max_left_blue_y)  

   ugv_left_obstacle1_right = math.hypot(max_left_red_x - max_right_blue_x, max_left_red_y - max_right_blue_y)  
  
   obstacle1_left_obstacle2_left = math.hypot(max_left_blue_x - max_left_green_x, max_left_blue_y - max_left_green_y)
  
   obstacle1_left_obstacle2_right = math.hypot(max_left_blue_x - max_right_green_x, max_left_blue_y - max_right_green_y) 
  
   obstacle1_right_obstacle2_left  = math.hypot(max_right_blue_x - max_left_green_x, max_right_blue_y - max_left_green_y) 
 
   obstacle1_right_obstacle2_right = math.hypot(max_right_blue_x - max_right_green_x, max_right_blue_y - max_right_green_y) 
 
   obstacle2_left_obstacle3_left = math.hypot(max_left_green_x - max_left_pink_x, max_left_green_y - max_left_pink_y)   

   obstacle2_left_obstacle3_right = math.hypot(max_left_green_x - max_right_pink_x, max_left_green_y - max_right_pink_y)  
 
   obstacle2_right_obstacle3_left = math.hypot(max_right_green_x - max_left_pink_x, max_right_green_y - max_left_pink_y)   

   obstacle2_right_obstacle3_right = math.hypot(max_right_green_x - max_right_pink_x, max_right_green_y - max_right_pink_y)
 
   obstacle3_right_end = math.hypot(max_right_pink_x - xc_orange, max_right_pink_y-yc_orange)

   obstacle3_left_end = math.hypot(max_left_pink_x - xc_orange, max_left_pink_y-yc_orange)

   obstacle1_relative_distance= math.hypot(xc_blue-xc_orange, yc_blue-yc_orange) 

   obstacle2_relative_distance= math.hypot(xc_green-xc_orange, yc_green-yc_orange) 

   obstacle3_relative_distance= math.hypot(xc_pink-xc_orange, yc_pink-yc_orange) 

   ugv_relative_distance= math.hypot(xc_red-xc_orange, yc_red-yc_orange) 

   ugv_to_blue = math.hypot(ugv_front_x - xc_blue, ugv_front_y - yc_blue)

   ugv_to_green = math.hypot(ugv_front_x - xc_green, ugv_front_y - yc_green)

   ugv_to_pink = math.hypot(ugv_front_x - xc_pink, ugv_front_y - yc_pink)

def fill_tree():
	calculate_multipliers()
	distances = {"C1":((ugv_left_obstacle1_right*M1)+(obstacle1_right_obstacle2_right*M2)+(obstacle2_right_obstacle3_right*M3)+obstacle3_right_end),
                     "C2":((ugv_left_obstacle1_right*M1)+(obstacle1_right_obstacle2_right*M2)+(obstacle2_right_obstacle3_left*M3)+obstacle3_left_end), 
		     "C3":((ugv_left_obstacle1_right*M1)+(obstacle1_right_obstacle2_left*M2)+(obstacle2_left_obstacle3_right*M3)+obstacle3_right_end), 
		     "C4":((ugv_left_obstacle1_right*M1)+(obstacle1_right_obstacle2_left*M2)+(obstacle2_left_obstacle3_left*M3)+obstacle3_left_end), 
		     "C5":((ugv_right_obstacle1_left*M1)+(obstacle1_left_obstacle2_right*M2)+(obstacle2_right_obstacle3_right*M3)+obstacle3_right_end), 
		     "C6":((ugv_right_obstacle1_left*M1)+(obstacle1_left_obstacle2_right*M2)+(obstacle2_right_obstacle3_left*M3)+obstacle3_right_end), 
		     "C7":((ugv_right_obstacle1_left*M1)+(obstacle1_left_obstacle2_left*M2)+(obstacle2_left_obstacle3_right*M3)+obstacle3_right_end), 
		     "C8":((ugv_right_obstacle1_left*M1)+(obstacle1_left_obstacle2_left*M2)+(obstacle2_left_obstacle3_left*M3)+obstacle3_left_end)}
        for leaf in unrooted_tree:
           leaf.add_features(distance=distances.get(leaf.name, "none"))

def send():
	f = open('/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/communication.txt', 'w')
	sent = 2
	if directions in ['C1', 'C2', 'C3', 'C4']:
		obstacle1 = 0
	if directions in ['C5', 'C6', 'C7', 'C8']:
		obstacle1 = 1
	if directions in ['C1', 'C2', 'C5', 'C6']:
		obstacle2 = 0
	if directions in ['C3', 'C4', 'C7', 'C8']:
		obstacle2 = 1
	if directions in ['C1', 'C3', 'C5', 'C7']:
		obstacle3 = 0
	if directions in ['C2', 'C4', 'C6', 'C8']:
		obstacle3 = 1
	f.write(str(sent))
	f.write(" ")
	f.write(str(obstacle1))
	f.write(" ")
	f.write(str(obstacle2))
	f.write(" ")
	f.write(str(obstacle3))
	f.write(str(ugv_to_blue))
	f.write(str(ugv_to_green))
	f.write(str(ugv_to_pink))
	f.close()

def calculate_smallest():
	########## TEST ##################
        smallest = 9999999
	global directions
	for leaf in unrooted_tree:
 		 if(leaf.distance<smallest):
			smallest = leaf.distance
			directions = leaf.name

def calculate_multipliers():
	global M1
	global M2
	global M3
	if(obstacle1_relative_distance > ugv_relative_distance):
		M1 = 0
	else:
		M1 = 1

	if(obstacle2_relative_distance > ugv_relative_distance):
		M2 = 0
	else:
		M2 = 1

	if(obstacle3_relative_distance > ugv_relative_distance):
		M3 = 0
	else:
		M3 = 1

class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = sys.maxint
        # Mark all nodes unvisited        
        self.visited = False  
        # Predecessor
        self.previous = None

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()  

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

def shortest(v, path):
    ''' make shortest path from v.previous'''
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return
 
def dijkstra(aGraph, start, target):
    #print '''Dijkstra's shortest path'''
    # Set the distance for the start node to zero 
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(),v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance 
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        #for next in v.adjacent:
        for next in current.adjacent:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)
            
            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
            #     print 'updated : current = %s next = %s new_dist = %s' \
            #             %(current.get_id(), next.get_id(), next.get_distance())
            # else:
            #     print 'not updated : current = %s next = %s new_dist = %s' \
            #             %(current.get_id(), next.get_id(), next.get_distance())

        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(),v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)

def send_dijkstra(path):
	f = open('/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/communication.txt', 'w')
	d = open('/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/distances.txt', 'w')
	sent = 2
	if "1" in path :
		obstacle1 = 0
	if "2" in path:
		obstacle1 =1
	if "3" in path :
		obstacle2 = 0
	if "4" in path:
		obstacle2 =1
	if "5" in path :
		obstacle3 = 0
	if "6" in path:
		obstacle3 =1
	f.write(str(sent))
	f.write(" ")
	f.write(str(obstacle1))
	f.write(" ")
	f.write(str(obstacle2))
	f.write(" ")
	f.write(str(obstacle3))
	d.write(str(ugv_to_blue))
	d.write(" ")
	d.write(str(ugv_to_green))
	d.write(" ")
	d.write(str(ugv_to_pink))
	f.close()
	d.close()
if __name__ == '__main__':
    sonarquad1()
    uav_image_red_2()
    #uav_image_orange()
    #uav_image_blue()
    #uav_image_green()
    #uav_image_pink()
    imuquad1()
    ugvnav()
    QuaterniontoEuler()
    try:
        time.sleep(2)
    	takeoff(7)
        hover(50000)
        track_ugv_2(4000000)

	
	
	
    except rospy.ROSInterruptException: pass
