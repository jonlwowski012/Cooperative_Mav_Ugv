#!/usr/bin/env python

# Quadrotor Simulator
# Jonathan Lwowski and Yonggun Lee
# Email: jonathan.lwowski@gmail.com yonggun.lee@utsa.edu
# Unmanned Systems Lab
# The University of Texas at San Antonio

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
bearing=0
bearing2=0
phi=0
theta=0
psi=0
nxc_red = 0
nyc_red = 0
xc_red = 0
yc_red = 0
ugv_yaw = 0


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image",Image)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    (rows,cols,channels) = cv_image.shape
    
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
    ### End of Image Processing ######
    cv2.imshow('Image window', cv_image)
    cv2.waitKey(3)

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      
    except CvBridgeError, e:
      print e

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
    quad1sonar = data


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
   

def imuquad1():
    rospy.Subscriber("/ardrone/imu", Imu, callback5)  


def sonarquad1():
    rospy.Subscriber("/sonar_height", Range, callback12) 



def pressurequad1():
    rospy.Subscriber("/pressure_height", PointStamped, callback16)



def magneticquad1():
    rospy.Subscriber("/ardrone/mag", Vector3Stamped, callback20)

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

def ugvnav():
    rospy.Subscriber("/p3dx/odom", Odometry, callback21)


def takeoff():
    pub1 = rospy.Publisher('/ardrone/takeoff', Empty)
    rospy.init_node('takeoff', anonymous=True)
    r = rospy.Rate(10) # 10hz
    x = 0
    for x in range(0, 700000):
        pub1.publish(Empty())


def hover(time):

    pub1 = rospy.Publisher('/cmd_vel', Twist)
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
       



def land():
    pub1 = rospy.Publisher('/ardrone/land', Empty)
    #rospy.init_node('takeoff', anonymous=True)
    r = rospy.Rate(10) # 10hz
    command = Empty()
    c = 0
    for c in range(0, 400000):
        pub1.publish(command)
     
  
def sepmovetime(time, lx1, ly1, lz1, ax1, ay1, az1):   
    pub1 = rospy.Publisher('/cmd_vel', Twist)


   
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

def sepmove(lx1, ly1, lz1, ax1, ay1, az1):   
    pub1 = rospy.Publisher('/cmd_vel', Twist)
    rospy.init_node('takeoff', anonymous=True)
    r = rospy.Rate(10) # 10hz
    command1 = Twist()
    command1.linear.x = lx1
    command1.linear.y = ly1
    command1.linear.z = lz1
    command1.angular.x = ax1
    command1.angular.y = ay1
    command1.angular.z = az1
    pub1.publish(command1)

def set_velocity_uav(lx1, ly1, lz1, ax1, ay1, az1):   
    pub1 = rospy.Publisher('/cmd_vel', Twist)


   
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
    
def takeoff_height(height):
    pub1 = rospy.Publisher('/cmd_vel', Twist)
    rospy.init_node('takeoff', anonymous=True)
    r = rospy.Rate(10) # 10hz
    command = Twist()
    command.linear.x = 0.00
    command.linear.y = 0.00
    command.linear.z = .3
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


        
if __name__ == '__main__':
    image_converter()
    sonarquad1()
    imuquad1()
    QuaterniontoEuler()
    
    try:
		takeoff()
		takeoff_height(10)
		hover(100000)
		track_ugv_2(100000)
		hover(100000)
		land()
	
	
	
    except rospy.ROSInterruptException: pass
