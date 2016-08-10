#!/usr/bin/env python

# Quadrotor Simulator
# Jonathan Lwowski 
# Email: jonathan.lwowski@gmail.com
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
import time
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
import random

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
bearing_blue=0
bearing_green=0
phi=0
theta=0
psi=0
nxc_blue=0
nyc_blue=0
nxc_pink=0
nyc_pink=0
nxc_green=0
nyc_green=0
ugv_yaw=0
ugv_orientation_x=0
ugv_orientation_y=0
ugv_orientation_z=0
ugv_orientation_w=0
ugv_pose_x=0
ugv_pose_y=0
detected = 0
direction = 0
bearing_right_edge = 0
bearing_left_edge = 0


class ugv_image_blue:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/p3dx/front_camera/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
    ### Start of Image Processing ######

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mat=cv.GetMat(cv.fromarray(mask))
    moments=cv.Moments(mat)
    if((moments.m01>(-10000000000000)) and (moments.m01<10000000000000) and (moments.m00>(-1000000000000)) and (moments.m00<10000000000000) and (moments.m10>(-1000000000000)) and (moments.m10<10000000000000)):
        global detected
        mean = mask.mean()
        if(mean > 0.5):
           detected = 1
        if(mean < 0.5 and detected == 1):
           detected = 0
        yc= moments.m01/moments.m00
        xc=moments.m10/moments.m00
        width, height = cv.GetSize(mat)
        global max_right_blue_x
        global max_left_blue_x
        global max_right_blue_y
        global max_left_blue_y

        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        cnt = contours[0]
        #print contours

        leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
        rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
        max_left_blue_x = leftmost[0]
        max_left_blue_y = leftmost[1]
        max_right_blue_x = rightmost[0]
        max_right_blue_y = rightmost[1]
        global nxc_blue
        global nyc_blue
        nxc_blue=xc-640
        nyc_blue=yc-360
        new_max_left_blue_x = max_left_blue_x -640
        new_max_left_blue_y = max_left_blue_y - 360
        new_max_right_blue_x = max_right_blue_x - 640
        new_max_right_blue_y = max_right_blue_y - 360
	focal=1097.51
	q= nxc_blue/focal
    q1 = new_max_left_blue_x / focal
    q2 = new_max_right_blue_x / focal
        global bearing_blue
        global bearing_left_edge
        global bearing_right_edge
        bearing_right_edge = math.atan(q2)*((180.0)/(3.14159))
        bearing_left_edge = math.atan(q1)*((180.0)/(3.14159))
        bearing_blue=math.atan(q)*((180.0)/(3.14159))
        cv2.circle(cv_image,(int(xc),int(yc)),10,(0,0,255),-1)
        #cv2.imshow('Thresholded', mask)
        

    ### End of Image Processing ######
    cv2.imshow('UAV Image Blue', cv_image)
    cv2.waitKey(3)

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      
    except CvBridgeError, e:
      print e


def callback21(data):
    global ugv_orientation_x
    global ugv_orientation_y
    global ugv_orientation_z
    global ugv_orientation_w
    global ugv_pose_x
    global ugv_pose_y
    ugv_pose_x = data.pose.pose.position.x
    ugv_pose_y = data.pose.pose.position.y
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



def ugvnav():
    rospy.Subscriber("/p3dx/odom", Odometry, callback21)    
   
  

def set_velocity_ugv(lx1, ly1, lz1, ax1, ay1, az1):   
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

    pub1.publish(command1)
	


    

def bearingangleavoidance_blue():
  V=.3;
  g = -9.8
  k = 1
  direction = 1
  if (direction == 1) :
    nhat=bearing_left_edge
  else:
    nhat = bearing_right_edge
  n = bearing_blue
  eta_d1 = 25*(3.14159/180)
  nd = direction * abs(eta_d1)
  u_psi = (abs(V*math.sin(bearing_blue*(3.14159/180)))/rho_tilde_min+B0)*satSgn((bearing_blue*(3.14159/180)-eta_d2)/epsilon)
  set_velocity_ugv(.3,0,0,0,0,u_psi)


	   
 
	
def satSgn(u): 
    if (u>-1 and u <1):
    	output = u;
    else:
    	output = sign(u) 
    return output
    
def sign(u): 
    if (u>0):
    	output = 1
    elif (u<0):
    	output = -1
    elif (u==0):
    	output=0 
    return output 
 		
if __name__ == '__main__':
  ugv_image_blue()
  ugvnav()
  try:	  
    time.sleep(2)
    delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    spawn = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    f = open('/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/urdf/blue_cylinder.urdf','r')
    sdff_blue = f.read()
    pose = Pose()
	############ Delete All ##############################
    delete("unit_cylinder_1_1")
    delete("unit_cylinder_2_0")
    delete("unit_cylinder_3")
    delete("unit_box_1_0")
    delete("unit_cylinder_4")
    delete("unit_cylinder_5")
    delete("unit_cylinder_6")
    delete("unit_box_2")
    delete("unit_cylinder_7")
    delete("unit_cylinder_8")
    delete("unit_cylinder_9")
    delete("unit_box_3")
    delete("unit_cylinder_1")
    delete("unit_cylinder_2")
    delete("unit_cylinder_1_0")
    delete("unit_box_1")
    ############# Spawn Set 1 ################################  
    pose.position.x = 7
    pose.position.y = random.uniform(-1,1)
    pose.position.z = .5
    spawn("blue_cylinder_1", sdff_blue, "urdf", pose, "world")
    while (1):
      bearingangleavoidance_blue()




  except rospy.ROSInterruptException: pass
