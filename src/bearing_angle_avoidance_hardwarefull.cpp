#include "ros/ros.h"
#include <ros/network.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <time.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using namespace std;

//******* Global Publishers   ***********//
ros::Publisher cmd_pub;

//******* Global Subscribers   ***********//
ros::Subscriber odom_sub;

//******* Global Variables   **********//
double ugv_pose_x;
double ugv_pose_y;
double ugv_yaw;
double bearing_blue;
double bearing_green;
double bearing_pink;
double bearing_yellow;
double direction;
double obstacle1_passed=1;
double obstacle2_passed=1;
double obstacle3_passed=1;
double door_seen = 0;
int objects_detected = 0;
int uav_in_room = 0;
cv::Scalar mean_blue;
cv::Scalar mean_green;
cv::Scalar mean_pink;
cv::Scalar mean_yellow;
ofstream myfile2;

//****** Image Processing Classes  *********//
class ImageConverter{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	//********    Start Image Processing     **********
	int xc_blue;
	int yc_blue;
	int xc_yellow;
	int yc_yellow;
	int xc_green;
	int yc_green;
	int xc_pink;
	int yc_pink;
	int nxc_blue;
	int nyc_blue;
	int nxc_yellow;
	int nyc_yellow;
	int nxc_green;
	int nyc_green;
	int nxc_pink;
	int nyc_pink;
	cv::Mat  hsv;
	cv::Mat  temp;
	cv::Mat  blueThreshed;
	cv::Mat  greenThreshed;
	cv::Mat  pinkThreshed;
	cv::Mat  yellowThreshed;
	cv::Mat  yellowThreshedinverted;
	cv::Moments moment;
	double q;
	double qr;
	double ql;

	double focal=1097.51;
	
	cv::Scalar lower_blue = cvScalar(100, 155, 65);
	cv::Scalar upper_blue = cvScalar(130, 255, 255);
	cv::Scalar lower_green = cvScalar(45, 100, 100);
	cv::Scalar upper_green = cvScalar(75, 255, 255);
	cv::Scalar lower_pink = cvScalar(10, 170, 150);
	cv::Scalar upper_pink = cvScalar(25, 255, 255);
	cv::Scalar lower_yellow = cvScalar(20, 0, 0);
	cv::Scalar upper_yellow = cvScalar(30, 255, 255);
	//*****  Threshold Image    ********** //
	
	cvtColor(cv_ptr->image, hsv , CV_BGR2HSV);
	inRange(hsv, lower_blue, upper_blue, blueThreshed);
	inRange(hsv, lower_green, upper_green, greenThreshed);
	inRange(hsv, lower_pink, upper_pink, pinkThreshed);
	inRange(hsv, lower_yellow, upper_yellow, yellowThreshed);
	mean_yellow = mean(yellowThreshed);
	mean_blue = mean(blueThreshed);
	mean_green = mean(greenThreshed);
	mean_pink = mean(pinkThreshed);
	
	if(mean_blue[0] > 0.1){		
		moment = moments(blueThreshed);
		//*****  Find Centroid and Edges    ********** //
		yc_blue= (moment.m01/moment.m00);
		xc_blue= (moment.m10/moment.m00);
		nyc_blue= (moment.m01/moment.m00)-(blueThreshed.rows/2);
		nxc_blue= (moment.m10/moment.m00)-(blueThreshed.cols/2);
		
		//*****  Calculate Bearing Angles   ********** //
		q= nxc_blue/focal;
		bearing_blue=atan(q)*((180.0)/(3.14159));
		
		//*****  Draw Center on Image    ******* //
		circle(cv_ptr->image, Point(int(nxc_blue+(blueThreshed.cols/2)),int(nyc_blue+(blueThreshed.rows/2))), 10, (90,0,255));
	}
	if(mean_green[0] > 0.1){		
		moment = moments(greenThreshed);
		//*****  Find Centroid and Edges    ********** //
		yc_green= (moment.m01/moment.m00);
		xc_green= (moment.m10/moment.m00);
		nyc_green= (moment.m01/moment.m00)-(greenThreshed.rows/2);
		nxc_green= (moment.m10/moment.m00)-(greenThreshed.cols/2);
		
		//*****  Calculate Bearing Angle   ********** //
		q= nxc_green/focal;
		bearing_green=atan(q)*((180.0)/(3.14159));
		
		//*****  Draw Center on Image    ******* //
		circle(cv_ptr->image, Point(int(nxc_green+(greenThreshed.cols/2)),int(nyc_green+(greenThreshed.rows/2))), 10, (180,0,255));
	}	
    if(mean_pink[0] > 0.1){		
		moment = moments(pinkThreshed);
		//*****  Find Centroid and Edges    ********** //
		yc_pink= (moment.m01/moment.m00);
		xc_pink= (moment.m10/moment.m00);
		nyc_pink= (moment.m01/moment.m00)-(pinkThreshed.rows/2);
		nxc_pink= (moment.m10/moment.m00)-(pinkThreshed.cols/2);
		
		//*****  Calculate Bearing Angle    ********** //
		q= nxc_pink/focal;
		bearing_pink=atan(q)*((180.0)/(3.14159));
		
		//*****  Draw Center on Image    ******* //
		circle(cv_ptr->image, Point(int(nxc_pink+(pinkThreshed.cols/2)),int(nyc_pink+(pinkThreshed.rows/2))), 10, (180,0,255));
	}	
	if(mean_yellow[0] > 0.1){
		objects_detected = 0;
		vector<vector<Point> > contours_yellow;
		vector<Vec4i> hierarchy;
		temp = yellowThreshed.clone();
		findContours(temp, contours_yellow, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
		for(unsigned int i=0;i<contours_yellow.size();i++){
			if(contourArea(contours_yellow[i])>50){
				objects_detected = objects_detected + 1;
			}
		}
		door_seen = 1;
		yellowThreshedinverted = yellowThreshed.clone();
		yellowThreshedinverted =  cv::Scalar::all(255) - yellowThreshedinverted;
		moment = moments(yellowThreshedinverted);
		//*****  Find Centroid and Edges    ********** //
		yc_yellow= (moment.m01/moment.m00);
		xc_yellow= (moment.m10/moment.m00);
		nyc_yellow= (moment.m01/moment.m00)-(yellowThreshedinverted.rows/2);
		nxc_yellow= (moment.m10/moment.m00)-(yellowThreshedinverted.cols/2);
		
		//*****  Calculate Bearing Angles    ********** //
		q= nxc_yellow/focal;
		bearing_yellow=atan(q)*((180.0)/(3.14159));
		
		//*****  Draw Centers and Edges on Image    ******* //
		circle(cv_ptr->image, Point(int(nxc_yellow+(yellowThreshedinverted.cols/2)),int(nyc_yellow+(yellowThreshedinverted.rows/2))), 10, (180,0,255));
	}
    
    // Update GUI Window
    //cv::imshow("PINK", pinkThreshed);
    //cv::imshow("Blue", blueThreshed);
    cv::imshow("Green", cv_ptr->image);
    cv::waitKey(1);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
}; 

//******** Publisher Functions   ***********//
void set_velocity_ugv(const double lx1, const double az1){
    geometry_msgs::Twist command1;
    command1.linear.x = lx1;
    command1.angular.z = az1;
    cmd_pub.publish(command1);
}


//******** Subscriber Callbacks  ***********//
void odomCallback(const nav_msgs::Odometry& msg){
    ugv_pose_x = msg.pose.pose.position.x;
    ugv_pose_y = msg.pose.pose.position.y;
    
    double ugv_orientation_x=msg.pose.pose.orientation.x;
    double ugv_orientation_y=msg.pose.pose.orientation.y;
    double ugv_orientation_z=msg.pose.pose.orientation.z;
    double ugv_orientation_w=msg.pose.pose.orientation.w;
    double e1=ugv_orientation_x;
    double e2=ugv_orientation_y;
    double e0=ugv_orientation_z;
    double e3=ugv_orientation_w;
    
    ugv_yaw = atan2(2*(e0*e3+e1*e2),(pow(e0,2)+pow(e1,2)-pow(e2,2)-pow(e3,2)));
    
	
	
	//cout << ugv_pose_x << "," << ugv_pose_y << "," << ugv_yaw << endl;
    ugv_yaw=ugv_yaw*180/M_PI;
    
    myfile2 << ugv_pose_x << "," << ugv_pose_y << "," << ugv_yaw << endl;
}


//******** Other Functions   ************* //
double sign(double u){ 
    double output;
    if (u>0){
        output = 1;
	}
    else if (u<0){
        output = -1;
	}
    else if (u==0){
        output=0;
	}
    return output;
} 
double satSgn(double u){ 
	double output;
    if (u>-1 && u <1){
        output = u;
	}
    else{
        output = sign(u);
	} 
    return output;
}
void bearingangleavoidance_blue(){
	double V=.05;
	double rho_tilde_min = .02;
	double B0 = 0.01;
	double epsilon = .01;
	double eta_d1 = (12 * M_PI)/180;
	double eta_d2 = direction * abs(eta_d1);
	double u_psi = ((abs(V*sin(bearing_blue*(M_PI/180) - eta_d2))/rho_tilde_min)+B0)*satSgn(((bearing_blue*(M_PI/180))-eta_d2)/epsilon);
	set_velocity_ugv(.3, -u_psi);
	//cout << "upsi= " << u_psi << endl;
	
}
void bearingangleavoidance_green(){
	double V=.05;
	double rho_tilde_min = .02;
	double B0 = 0.01;
	double epsilon = .01;
	double eta_d1 = (12 * M_PI)/180;
	double eta_d2 = direction * abs(eta_d1);
	double u_psi = ((abs(V*sin(bearing_green*(M_PI/180) -eta_d2))/rho_tilde_min)+B0)*satSgn(((bearing_green*(M_PI/180))-eta_d2)/epsilon);
	set_velocity_ugv(.3, -u_psi);
	//cout << "upsi= " << u_psi << endl;
}
void bearingangleavoidance_pink(){
	double V=.05;
	double rho_tilde_min = .02;
	double B0 = 0.01;
	double epsilon = .01;
	double eta_d1 = (12 * M_PI)/180;
	double eta_d2 = direction * abs(eta_d1);
	double u_psi = ((abs(V*sin(bearing_pink*(M_PI/180)-eta_d2))/rho_tilde_min)+B0)*satSgn(((bearing_pink*(M_PI/180))-eta_d2)/epsilon);
	//cout << "upsi= " << u_psi << endl;
	set_velocity_ugv(.3, -u_psi);
}
double rotate_target(double x, double y){
	double angle;
	if(x<0 && y >0){
		angle = atan2(x, y) + M_PI/2;
	}
	else if(x>0 && y>0){
		angle = atan2(x, y) + M_PI/2;
	}
	else if(x<0 && y<0){
		angle = atan2(x, y)+ M_PI/2;
	}
	else if(x>0 && y<0){  
		angle =  atan2(x, y) - (3*M_PI/2);
	}
	else if(x==0 && y>0){
		angle = M_PI/2;
	}
	else if(x==0 && y<0){
		angle = -M_PI/2;
	}
	else if(y==0 && x<0){
		angle = 0;
	}
	else if(y==0 && x>0){
		angle = M_PI;
	}
	angle=angle*180/M_PI;
	return angle;
}
double angle_difference(double uav_angle, double ugv_angle){
	double diff;
	if(uav_angle >=0 && ugv_angle >=0){
		diff=abs(uav_angle - ugv_angle);
	}
	else if (uav_angle >=0 && ugv_angle <0){
		diff = abs(360-uav_angle-abs(ugv_angle));
	}
	else if(uav_angle <0 && ugv_angle >=0){
		diff = abs( abs(uav_angle) + ugv_angle);
	}
	else if(uav_angle <0 && ugv_angle < 0){
		diff = abs(uav_angle - ugv_angle);
	}
	if(diff > 180){
		diff = abs(360-diff);
	}
   return diff;
} 
double turn_direction(double angle_init, double ugv_angle){
	double angle_back;
	double turn_sign;
	if(angle_init >= 0){
		angle_back = angle_init-180;
	}
	else{
		angle_back = angle_init+180;
	}
	if(angle_init >=0 && ugv_angle <= angle_init && ugv_angle >= angle_back){
		turn_sign = 1;
	}
	else if(angle_init >=0 && ((ugv_angle >= angle_init && ugv_angle <=180) || (ugv_angle <= angle_back))){
		turn_sign = -1;
	}
	else if(angle_init < 0 && ((ugv_angle <= angle_init && ugv_angle >=-180) || (ugv_angle >= angle_back))){
		turn_sign = 1;
	}
	else if (angle_init < 0 && ugv_angle > angle_init && ugv_angle < angle_back){
		turn_sign = -1;
	}
   return turn_sign;
} 
double goto_target(double x, double y){
	ros::Rate loop_rate(50);
	loop_rate.sleep();
	ros::spinOnce();
	double D_yaw = 1;   
	double Derivator_yaw = 0;
	double error_yaw = 0;
	double Kd_yaw = 1;
	double diff_x = abs(ugv_pose_x-x);
	double diff_y = abs(ugv_pose_y-y);
	double goal_x = x - ugv_pose_x;
	double goal_y = y - ugv_pose_y;
	double angle;
	double diff_yaw;
	double sign_yaw;
	double P_yaw;
	double PD_yaw;
	
	while (diff_x > .1 || diff_y>.1){
		angle = rotate_target(goal_x,goal_y);
		// ******** YAW PID  ********* //
		diff_yaw = angle_difference(ugv_yaw,angle);
		sign_yaw = turn_direction(ugv_yaw, angle);
		P_yaw = sign_yaw*diff_yaw*.05;
		error_yaw = diff_yaw;
		D_yaw = 0.05 * (error_yaw - Derivator_yaw);
		PD_yaw = P_yaw - D_yaw;
		Derivator_yaw= error_yaw;
		Kd_yaw = -D_yaw;
		set_velocity_ugv(.4,PD_yaw);
		diff_x = abs(ugv_pose_x-x);
		diff_y = abs(ugv_pose_y-y);
		//cout<<"diff x" << ugv_pose_x << " diff y "<< ugv_pose_y << endl;
		goal_x = x - ugv_pose_x;
		goal_y = y - ugv_pose_y;
		if(mean_blue[0] > 0.1 || mean_green[0] > 0.1 || mean_pink[0] > 0.1){
			return 0;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	set_velocity_ugv(0,0);
	return 1;
}
void reset_communication(){
	obstacle1_passed=1;
	obstacle2_passed=1;
	obstacle3_passed=1;
	ofstream myfile;
	double sentcheck = 0;
	myfile.open ("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/communication.txt");
	myfile << (double)sentcheck << endl;
	myfile << (double)sentcheck << endl;
	myfile << (double)sentcheck << endl;
	myfile << (double)sentcheck << endl;
	myfile.close();
}
void recieve_directions(double* array){
	std::ifstream ifile("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/communication.txt", std::ios::in);
	std::vector<double> scores;
	//check to see that the file was opened correctly:
    if (!ifile.is_open()) {
        std::cerr << "There was a problem opening the input file!\n";
        exit(1);//exit or do additional error checking
    }

    double num = 0.0;
    //keep storing values from the text file so long as data exists:
    while (ifile >> num) {
        scores.push_back(num);
    }

    //verify that the scores were stored correctly:
    for (int i = 0; i < scores.size(); ++i) {
        array[i] = scores[i];
    }
    
}
void maincontroller(double x, double y){
	ros::Rate loop_rate(100);
	double sentcheck = 1;
	double finished=0;
	double directions[4];
	double ispassed[3];
	reset_communication();
	recieve_directions(&directions[0]);
	
	
	while(directions[0] != sentcheck){
		recieve_directions(&directions[0]);
		loop_rate.sleep();
		ros::spinOnce();
		}
		
		
	while (finished != 1){
		if(obstacle1_passed != 0){
			obstacle1_passed = ispassed[0];
		}
		if(obstacle2_passed != 0){
			obstacle2_passed = ispassed[1];
		}
		if(obstacle3_passed != 0){
			obstacle3_passed = ispassed[2];
		}
		loop_rate.sleep();
		ros::spinOnce();
		
		if(mean_blue[0] > 1 && obstacle1_passed == 1){
			direction = directions[1];
			bearingangleavoidance_blue();
		}
		else if(mean_green[0] > 1 && obstacle2_passed==1){
			direction = directions[2];
			bearingangleavoidance_green();
		}
		else if(mean_pink[0] > 1 && obstacle3_passed==1){
			direction = directions[3];
			bearingangleavoidance_pink();
		}
		else{
			finished = goto_target(x,y);
		}
	}	
}
void no_canopy_controller(){	
		/***********  Calling Controller    *******/
		maincontroller(10,0);
}    

int main(int argc, char **argv){    
	ros::init(argc, argv, "ugv");
	ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ImageConverter ic;
    
    myfile2.open ("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/ugv_path.csv");
    //   Publishers
    cmd_pub = n.advertise<geometry_msgs::Twist>("/p2os/cmd_vel", 100);
    
    //  Subscribers
    odom_sub=n.subscribe("/p2os/pose", 100, odomCallback);
    
    
    // Main Code
    while (ros::ok())
    { 
		no_canopy_controller();
        loop_rate.sleep();
        ros::spinOnce();
        break;
        
    }

    
    myfile2.close();
    return 0;
}
