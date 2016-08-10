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
#include <visualization_msgs/Marker.h>
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

/******** Global Spawner and Deleter Stuff *******/
ros::ServiceClient deleter;
ros::ServiceClient spawner;
gazebo_msgs::DeleteModelRequest dreq;
gazebo_msgs::DeleteModelResponse dresp;
gazebo_msgs::SpawnModelRequest req;
gazebo_msgs::SpawnModelResponse resp;
std::string modelString_blue;
std::string modelString_green;
std::string modelString_pink;
std::string modelString_orange;

//******* Global Publishers   ***********//
ros::Publisher cmd_pub;

//******* Global Subscribers   ***********//
ros::Subscriber odom_sub;
ros::Subscriber marker_sub;

//******* Global Variables   **********//
double ugv_pose_x;
double ugv_pose_y;
double ugv_yaw;
double bearing_blue;
double bearing_green;
double bearing_pink;
double bearing_yellow;
double bearing_turquoise;
double direction;
double obstacle1_passed=1;
double obstacle2_passed=1;
double obstacle3_passed=1;
double nxc_yellow;
double nxc_turquoise;
double door_seen = 0;
int objects_detected = 0;
int uav_in_room = 0;
cv::Scalar mean_blue;
cv::Scalar mean_green;
cv::Scalar mean_pink;
cv::Scalar mean_yellow;
cv::Scalar mean_turquoise;
geometry_msgs::Pose marker_pose[10];

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
    image_sub_ = it_.subscribe("/p3dx/front_camera/image_raw", 1, 
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
	int xc_turquoise;
	int yc_turquoise;
	int xc_green;
	int yc_green;
	int xc_pink;
	int yc_pink;
	int nxc_blue;
	int nyc_blue;
	int nyc_yellow;
	int nyc_turquoise;
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
	cv::Mat turquoiseThreshed;
	cv::Moments moment;
	double q;
	double qr;
	double ql;

	double focal=1097.51;
	
	cv::Scalar lower_blue = cvScalar(110, 50, 50);
	cv::Scalar upper_blue = cvScalar(130, 255, 255);
	cv::Scalar lower_green = cvScalar(50, 50, 50);
	cv::Scalar upper_green = cvScalar(70, 255, 255);
	cv::Scalar lower_pink = cvScalar(143, 0, 0);
	cv::Scalar upper_pink = cvScalar(162, 255, 255);
	cv::Scalar lower_yellow = cvScalar(20, 0, 0);
	cv::Scalar upper_yellow = cvScalar(30, 255, 255);
	cv::Scalar lower_turquoise = cvScalar(90, 100, 100);
	cv::Scalar upper_turquoise = cvScalar(100, 255, 255);
	//*****  Threshold Image    ********** //
	
	cvtColor(cv_ptr->image, hsv , CV_BGR2HSV);
	inRange(hsv, lower_blue, upper_blue, blueThreshed);
	inRange(hsv, lower_green, upper_green, greenThreshed);
	inRange(hsv, lower_pink, upper_pink, pinkThreshed);
	inRange(hsv, lower_yellow, upper_yellow, yellowThreshed);
	inRange(hsv, lower_turquoise, upper_turquoise, turquoiseThreshed);
	mean_yellow = mean(yellowThreshed);
	mean_blue = mean(blueThreshed);
	mean_green = mean(greenThreshed);
	mean_pink = mean(pinkThreshed);
	mean_turquoise = mean(turquoiseThreshed);
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
		door_seen = 1;
		moment = moments(yellowThreshed);
		//*****  Find Centroid and Edges    ********** //
		yc_yellow= (moment.m01/moment.m00);
		xc_yellow= (moment.m10/moment.m00);
		nyc_yellow= (moment.m01/moment.m00)-(yellowThreshed.rows/2);
		nxc_yellow= (moment.m10/moment.m00)-(yellowThreshed.cols/2);
		
		//*****  Calculate Bearing Angles    ********** //
		q= nxc_yellow/focal;
		bearing_yellow=atan(q)*((180.0)/(3.14159));
		
		//*****  Draw Centers and Edges on Image    ******* //
		circle(cv_ptr->image, Point(int(nxc_yellow+(yellowThreshed.cols/2)),int(nyc_yellow+(yellowThreshed.rows/2))), 10, (180,0,255));
	}
    if(mean_turquoise[0] > 0.1){		
		moment = moments(turquoiseThreshed);
		//*****  Find Centroid and Edges    ********** //
		yc_turquoise= (moment.m01/moment.m00);
		xc_turquoise= (moment.m10/moment.m00);
		nyc_turquoise= (moment.m01/moment.m00)-(turquoiseThreshed.rows/2);
		nxc_turquoise = (moment.m10/moment.m00)-(turquoiseThreshed.cols/2);
		
		//*****  Calculate Bearing Angles    ********** //
		q= nxc_turquoise/focal;
		bearing_turquoise=atan(q)*((180.0)/(3.14159));
		
		//*****  Draw Centers and Edges on Image    ******* //
		circle(cv_ptr->image, Point(int(nxc_turquoise+(turquoiseThreshed.cols/2)),int(nyc_turquoise+(turquoiseThreshed.rows/2))), 10, (180,0,255));
	}
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
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
    ugv_yaw=ugv_yaw*180/M_PI;
}
void markerCallback(const visualization_msgs::Marker msg){
    geometry_msgs::Pose pose = msg.pose; 
    int id = msg.id; 
    marker_pose[id]=pose;
    cout<<"Pose: " << marker_pose[id] << " Id: " << id << endl;
}

//******** Other Functions   ************* //
float randbetween(float min, float max){
	return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;
}
void delete_obstacle_set_4(){
	dreq.model_name = "blue_cylinder_4";
	deleter.call(dreq,dresp);
	dreq.model_name ="green_cylinder_4";
	deleter.call(dreq,dresp);
	dreq.model_name ="pink_cylinder_4";
	deleter.call(dreq,dresp);
	dreq.model_name ="orange_box_4";
	deleter.call(dreq,dresp);
}
void spawn_obstacle_set_4(){
	req.model_name = "blue_cylinder_4";
	req.reference_frame = "world";
	req.model_xml = modelString_blue;
	req.initial_pose.position.x = randbetween(-2, 0);
	req.initial_pose.position.y = -8;
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
	req.model_name = "green_cylinder_4";
	req.reference_frame = "world";
	req.model_xml = modelString_green;
	req.initial_pose.position.x = randbetween(-2, 0);
	req.initial_pose.position.y = -5;
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
	req.model_name = "pink_cylinder_4";
	req.reference_frame = "world";
	req.model_xml = modelString_pink;
	req.initial_pose.position.x = randbetween(-2, 0);
	req.initial_pose.position.y = -2;
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
	req.model_name = "orange_box_4";
	req.reference_frame = "world";
	req.model_xml = modelString_orange;
	req.initial_pose.position.x = -1;
	req.initial_pose.position.y = 1;
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
}
void delete_obstacle_set_3(){
	dreq.model_name = "blue_cylinder_3";
	deleter.call(dreq,dresp);
	dreq.model_name ="green_cylinder_3";
	deleter.call(dreq,dresp);
	dreq.model_name ="pink_cylinder_3";
	deleter.call(dreq,dresp);
	dreq.model_name ="orange_box_3";
	deleter.call(dreq,dresp);
}
void spawn_obstacle_set_3(){
	req.model_name = "blue_cylinder_3";
	req.reference_frame = "world";
	req.model_xml = modelString_blue;
	req.initial_pose.position.x = 8;
	req.initial_pose.position.y = randbetween(-11, -13);
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
	req.model_name = "green_cylinder_3";
	req.reference_frame = "world";
	req.model_xml = modelString_green;
	req.initial_pose.position.x = 5;
	req.initial_pose.position.y = randbetween(-11, -13);
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
	req.model_name = "pink_cylinder_3";
	req.reference_frame = "world";
	req.model_xml = modelString_pink;
	req.initial_pose.position.x = 2;
	req.initial_pose.position.y = randbetween(-11, -13);
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
	req.model_name = "orange_box_3";
	req.reference_frame = "world";
	req.model_xml = modelString_orange;
	req.initial_pose.position.x = -1;
	req.initial_pose.position.y = -12;
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
}
void delete_obstacle_set_2(){
	dreq.model_name = "blue_cylinder_2";
	deleter.call(dreq,dresp);
	dreq.model_name ="green_cylinder_2";
	deleter.call(dreq,dresp);
	dreq.model_name ="pink_cylinder_2";
	deleter.call(dreq,dresp);
	dreq.model_name ="orange_box_2";
	deleter.call(dreq,dresp);	
}
void spawn_obstacle_set_2(){
	req.model_name = "blue_cylinder_2";
	req.reference_frame = "world";
	req.model_xml = modelString_blue;
	req.initial_pose.position.x = randbetween(11, 13);
	req.initial_pose.position.y = -4;
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
	req.model_name = "green_cylinder_2";
	req.reference_frame = "world";
	req.model_xml = modelString_green;
	req.initial_pose.position.x = randbetween(11, 13);
	req.initial_pose.position.y = -7;
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
	req.model_name = "pink_cylinder_2";
	req.reference_frame = "world";
	req.model_xml = modelString_pink;
	req.initial_pose.position.x = randbetween(11, 13);
	req.initial_pose.position.y = -9;
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
	req.model_name = "orange_box_2";
	req.reference_frame = "world";
	req.model_xml = modelString_orange;
	req.initial_pose.position.x = 12;
	req.initial_pose.position.y = -12;
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
}
void delete_obstacle_set_1(){
	dreq.model_name = "blue_cylinder_1";
	deleter.call(dreq,dresp);
	dreq.model_name ="green_cylinder_1";
	deleter.call(dreq,dresp);
	dreq.model_name ="pink_cylinder_1";
	deleter.call(dreq,dresp);
	dreq.model_name ="orange_box_1";
	deleter.call(dreq,dresp);
}
void spawn_obstacle_set_1(){
	req.model_name = "blue_cylinder_1";
	req.reference_frame = "world";
	req.model_xml = modelString_blue;
	req.initial_pose.position.x = 3;
	req.initial_pose.position.y = randbetween(-1, 1);
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
	req.model_name = "green_cylinder_1";
	req.reference_frame = "world";
	req.model_xml = modelString_green;
	req.initial_pose.position.x = 6;
	req.initial_pose.position.y = randbetween(-1, 1);
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
	req.model_name = "pink_cylinder_1";
	req.reference_frame = "world";
	req.model_xml = modelString_pink;
	req.initial_pose.position.x = 9;
	req.initial_pose.position.y = randbetween(-1, 1);
	req.initial_pose.position.z = .5;
	spawner.call(req,resp);
	req.model_name = "orange_box_1";
	req.reference_frame = "world";
	req.model_xml = modelString_orange;
	req.initial_pose.position.x = 12;
	req.initial_pose.position.y = 0;
	req.initial_pose.position.z = 0;
	spawner.call(req,resp);
}
void delete_obstacles_all(){
	dreq.model_name = "unit_cylinder_8";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_cylinder_1_1";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_cylinder_2_0";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_cylinder_3";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_box_1_0";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_cylinder_4";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_cylinder_5";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_cylinder_6";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_box_2";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_cylinder_7";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_cylinder_8";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_cylinder_9";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_box_3";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_cylinder_1";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_cylinder_2";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_cylinder_1_0";
	deleter.call(dreq,dresp);
	dreq.model_name ="unit_box_1";
	deleter.call(dreq,dresp);
}
std::string loadModelSDF(const std::string file){
	std::string filename;
	try
	{
		if (file.find("package://") == 0)
		{
			filename = file;
			filename.erase(0, strlen("package://"));
			size_t pos = filename.find("/");
			if (pos != std::string::npos)
			{
				std::string package = filename.substr(0, pos);
				filename.erase(0, pos);
				std::string package_path = ros::package::getPath(package);
				filename = package_path + filename;
			}
		}
		else
		{
			ROS_ERROR("Failed to locate file: %s", file.c_str());
			return "";
		}
	}
	catch (std::exception& e)
	{
		ROS_ERROR("Failed to retrieve file: %s", e.what());
		return "";
	}

	std::ifstream t(filename);
	std::string outstr;

	t.seekg(0, std::ios::end);
	outstr.reserve(t.tellg());
	t.seekg(0, std::ios::beg);

	outstr.assign((std::istreambuf_iterator<char>(t)),
				   std::istreambuf_iterator<char>());

	return outstr;
}
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
	double V=.5;
	double rho_tilde_min = 3;
	double B0 = 0.2;
	double epsilon = .01;
	double eta_d1 = (14 * M_PI)/180;
	double eta_d2 = direction * abs(eta_d1);
	double u_psi = ((abs(V*sin(bearing_blue*(M_PI/180)))/rho_tilde_min)+B0)*satSgn(((bearing_blue*(M_PI/180))-eta_d2)/epsilon);
	set_velocity_ugv(.3, -u_psi);
	
}
void bearingangleavoidance_green(){
	double V=.5;
	double rho_tilde_min = 3;
	double B0 = 0.2;
	double epsilon = .01;
	double eta_d1 = (14 * M_PI)/180;
	double eta_d2 = direction * abs(eta_d1);
	double u_psi = ((abs(V*sin(bearing_green*(M_PI/180)))/rho_tilde_min)+B0)*satSgn(((bearing_green*(M_PI/180))-eta_d2)/epsilon);
	set_velocity_ugv(.3, -u_psi);
}
void bearingangleavoidance_pink(){
	double V=.5;
	double rho_tilde_min = 3;
	double B0 = 0.2;
	double epsilon = .01;
	double eta_d1 = (14 * M_PI)/180;
	double eta_d2 = direction * abs(eta_d1);
	double u_psi = ((abs(V*sin(bearing_pink*(M_PI/180)))/rho_tilde_min)+B0)*satSgn(((bearing_pink*(M_PI/180))-eta_d2)/epsilon);
	set_velocity_ugv(.3, -u_psi);
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
void recieve_ispassed(double* array){
	std::ifstream ifile("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/ispassed.txt", std::ios::in);
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
	myfile.open ("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/uavinroom.txt");
	myfile << (double)sentcheck << endl;
	myfile.close();
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
		D_yaw = Kd_yaw * (error_yaw - Derivator_yaw);
		PD_yaw = P_yaw + D_yaw;
		Derivator_yaw= error_yaw;
		Kd_yaw = D_yaw;
		set_velocity_ugv(.6,PD_yaw);
		diff_x = abs(ugv_pose_x-x);
		diff_y = abs(ugv_pose_y-y);
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
		recieve_ispassed(&ispassed[0]);
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
		if(mean_blue[0] > 0.1 && obstacle1_passed == 1){
			direction = directions[1];
			bearingangleavoidance_blue();
		}
		else if(mean_green[0] > 0.1 && obstacle2_passed==1){
			direction = directions[2];
			bearingangleavoidance_green();
		}
		else if(mean_pink[0] > 0.1 && obstacle3_passed==1){
			direction = directions[3];
			bearingangleavoidance_pink();
		}
		else{
			finished = goto_target(x,y);
		}
	}	
}
void no_canopy_controller(){	
		
		/********* Reading Models for Objects   *************************/
		modelString_blue = loadModelSDF("package://cooperative_obstacle_avoidance/urdf/blue_cylinder.urdf");
		modelString_green = loadModelSDF("package://cooperative_obstacle_avoidance/urdf/green_cylinder.urdf");
		modelString_pink = loadModelSDF("package://cooperative_obstacle_avoidance/urdf/purple_cylinder.urdf");
		modelString_orange = loadModelSDF("package://cooperative_obstacle_avoidance/urdf/orange_box.urdf");
		
		/********* Deleting Obstacles   *************************/
		delete_obstacles_all();
		
		/********* Spawning Obstacle Set 1   *************************/
		spawn_obstacle_set_1();
		
		/***********  Calling Controller    *******/
		maincontroller(14,0);
		
		/********* Deleting Obstacle Set 1   *************************/
		delete_obstacle_set_1();
		
		/********* Spawning Obstacle Set 2   *************************/
		spawn_obstacle_set_2();
		
		/***********  Calling Controller    *******/
		maincontroller(14,-12);
		
		/********* Deleting Obstacle Set 2   *************************/
		delete_obstacle_set_2();
        
        /********* Spawning Obstacle Set 3   *************************/
		spawn_obstacle_set_3();
		
		/***********  Calling Controller    *******/
		maincontroller(1,-12);
		
		/********* Deleting Obstacle Set 3   *************************/
		delete_obstacle_set_3();
        
        /********* Spawning Obstacle Set 4   *************************/
		spawn_obstacle_set_4();
		
		/***********  Calling Controller    *******/
		maincontroller(1,1);
		
		/********* Deleting Obstacle Set 4   *************************/
		delete_obstacle_set_4();
}    
void is_door_seen(){
	ofstream myfile;
	myfile.open ("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/doorseen.txt");
	myfile << (double)door_seen << endl;
	myfile.close();
}
void is_uav_in_room(){
	std::ifstream ifile("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/uavinroom.txt", std::ios::in);
	std::vector<double> scores;
	//check to see that the file was opened correctly:
    if (!ifile.is_open()) {
        std::cerr << "There was a problem opening the input file!\n";
        exit(1);//exit or do additional error checking
    }

    double num = 0.0;
    //keep storing values from the text file so long as data exists:
    ifile >> num;
    
    if (num == 1){
		uav_in_room = 1; // doorway is seen by ugv
	} 
	else{
		uav_in_room = 0; // doorway is not seen by ugv
	}
}
void canopy_controller(){
	reset_communication();
	ros::Rate loop_rate(100);
	double diff_yaw;
	double sign_yaw;
	double P_yaw;
	double PD_yaw;
	double D_yaw = 1;   
	double Derivator_yaw = 0;
	double error_yaw = 0;
	double Kd_yaw = 1;
	double direction_door;
	double rotate;
	while(1){
		is_door_seen();
		is_uav_in_room();
		if(uav_in_room == 1 || uav_in_room == 0 ){
			loop_rate.sleep();
			ros::spinOnce();
			cout << "yellow: " << mean_yellow[0] << endl;
			cout << "blue: " << mean_turquoise[0] << endl;
			if(mean_yellow[0] > .1 && mean_turquoise[0] > .1){
				// ############# Roll PID      ##################### 
				double P_yaw=((.05)*((nxc_yellow+nxc_turquoise)/2));
				error_yaw = ((nxc_yellow+nxc_turquoise)/2);
				double D_yaw = Kd_yaw * (error_yaw - Derivator_yaw);
				double PD_yaw = P_yaw+D_yaw;
				Derivator_yaw = error_yaw;
				Kd_yaw = D_yaw;
				set_velocity_ugv(.5,-PD_yaw);
				cout << "yaw: " << PD_yaw << endl;
			}
			else if (mean_yellow[0] > 10 && mean_turquoise[0] < 10){
				set_velocity_ugv(.2,-.2);
			}
			else if (mean_yellow[0] < 10 && mean_turquoise[0] > 10){
				set_velocity_ugv(.2,.2);
					}
			else{
				//goto_target(5,0);
			}
		}
		else{
			set_velocity_ugv(0, 0);
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}
int main(int argc, char **argv){
	ros::init(argc, argv, "ugv");
	ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ImageConverter ic;
    
    //   Publishers
    cmd_pub = n.advertise<geometry_msgs::Twist>("/p3dx/cmd_vel", 100);
    
    //  Subscribers
    odom_sub=n.subscribe("/p3dx/odom", 100, odomCallback);
    marker_sub=n.subscribe("/visualization_marker",100,markerCallback);
    
    /********* Setting Up Deleter and Spawner   *************************/
	deleter = n.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
	spawner = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    
    // Main Code
    while (ros::ok())
    { 
		usleep(10000);
		canopy_controller();
        loop_rate.sleep();
        ros::spinOnce();
        
    }

    
    
    return 0;
}
