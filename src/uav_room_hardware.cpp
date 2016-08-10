#include "ros/ros.h"
#include <ros/network.h>
#include <opencv2/opencv.hpp>
//#include </opt/ros/hydro/include/opencv2/imgproc/imgproc.hpp>
//#include </opt/ros/hydro/include/opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <stdio.h>
#include <stdlib.h>
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
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using namespace std;

//******* Global Publishers   ***********//
ros::Publisher cmd_pub;
ros::Publisher takeoff_pub;
ros::Publisher land_pub;

//******* Global Subscribers   ***********//
ros::Subscriber sonar_sub;
ros::Subscriber odom_sub;
ros::Subscriber imu_sub;

//******* Global Variables   **********//
double sonar_height;


int xc_turquoise;
int yc_turquoise;

int nxc_turquoise;
int nyc_turquoise;

double bearing_turquoise;

cv::Scalar mean_turquoise;

int xc_red;
int yc_red;
int xc_yellow;
int yc_yellow;
int xc_yellowinv;
int yc_yellowinv;
int xc_blue;
int yc_blue;
int xc_green;
int yc_green;
int xc_pink;
int yc_pink;
int xc_orange;
int yc_orange;
int nxc_red;
int nyc_red;
int nxc_yellow;
int nyc_yellow;
int nxc_yellowinv;
int nyc_yellowinv;
int nxc_blue;
int nyc_blue;
int nxc_green;
int nyc_green;
int nxc_pink;
int nyc_pink;
int nxc_orange;
int nyc_orange;
double ugv_pose_x;
double ugv_pose_y;
double uav_pose_x;
double uav_pose_y;
double ugv_yaw;
double uav_yaw;
double ugv_right_obstacle1_left;   
double ugv_right_obstacle1_right;    
double ugv_left_obstacle1_left;   
double ugv_left_obstacle1_right;   
double obstacle1_left_obstacle2_left;  
double obstacle1_left_obstacle2_right;  
double obstacle1_right_obstacle2_left;  
double obstacle1_right_obstacle2_right; 
double obstacle2_left_obstacle3_left;  
double obstacle2_left_obstacle3_right;  
double obstacle2_right_obstacle3_left;  
double obstacle2_right_obstacle3_right;
double obstacle3_right_end; 
double obstacle3_left_end; 
double obstacle1_relative_distance;
double obstacle2_relative_distance;
double obstacle3_relative_distance;
double ugv_relative_distance;
double ugv_to_blue;
double ugv_to_green;
double ugv_to_pink;
double bearing_red_right;
double bearing_red_left;
double bearing_red;
double bearing_blue_right;
double bearing_blue_left;
double bearing_blue;
double bearing_green_right;
double bearing_green_left;
double bearing_green;
double bearing_pink_right;
double bearing_pink_left;
double bearing_pink;
double bearing_yellow;
double bearing_yellowinv;
Point ugv_leftmost;
Point ugv_rightmost;
Point blue_leftmost;
Point blue_rightmost;
Point green_leftmost;
Point green_rightmost;
Point pink_leftmost;
Point pink_rightmost;
typedef int vertex_t;
typedef double weight_t;
double obstacle1_direction;
double obstacle2_direction;
double obstacle3_direction;
double obstacle1_passed;
double obstacle2_passed;
double obstacle3_passed;
cv::Scalar mean_red;
cv::Scalar mean_blue;
cv::Scalar mean_green;
cv::Scalar mean_pink;
cv::Scalar mean_orange;
cv::Scalar mean_yellow;
double door_seen=0;
int uav_in_room = 0; 
ofstream myfile2;

//**********  Dijkstra Functions & Classes   *****//
struct edge {
    vertex_t target;
    weight_t weight;
    edge(vertex_t arg_target, weight_t arg_weight)
        : target(arg_target), weight(arg_weight) { }
};
typedef std::map<vertex_t, std::list<edge> > adjacency_map_t;
template <typename T1, typename T2>
struct pair_first_less{
    bool operator()(std::pair<T1,T2> p1, std::pair<T1,T2> p2)
    {
        return p1.first < p2.first;
    }
};
void DijkstraComputePaths(vertex_t source, adjacency_map_t& adjacency_map,std::map<vertex_t, weight_t>& min_distance, std::map<vertex_t, vertex_t>& previous){
    for (adjacency_map_t::iterator vertex_iter = adjacency_map.begin();
         vertex_iter != adjacency_map.end();
         vertex_iter++)
    {
        vertex_t v = vertex_iter->first;
        min_distance[v] = std::numeric_limits< double >::infinity();
    }
    min_distance[source] = 0;
    std::set< std::pair<weight_t, vertex_t>,
              pair_first_less<weight_t, vertex_t> > vertex_queue;
    for (adjacency_map_t::iterator vertex_iter = adjacency_map.begin();
         vertex_iter != adjacency_map.end();
         vertex_iter++)
    {
        vertex_t v = vertex_iter->first;
        vertex_queue.insert(std::pair<weight_t, vertex_t>(min_distance[v], v));
    }

    while (!vertex_queue.empty()) {
        vertex_t u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());

        // Visit each edge exiting u
        for (std::list<edge>::iterator edge_iter = adjacency_map[u].begin();
             edge_iter != adjacency_map[u].end();
             edge_iter++)
        {
            vertex_t v = edge_iter->target;
            weight_t weight = edge_iter->weight;
            weight_t distance_through_u = min_distance[u] + weight;
	    if (distance_through_u < min_distance[v]) {
	        vertex_queue.erase(std::pair<weight_t, vertex_t>(min_distance[v], v));

	        min_distance[v] = distance_through_u;
	        previous[v] = u;
	        vertex_queue.insert(std::pair<weight_t, vertex_t>(min_distance[v], v));
	    }
        }
    }
}
std::list<vertex_t> DijkstraGetShortestPathTo(vertex_t target, std::map<vertex_t, vertex_t>& previous){
    std::list<vertex_t> path;
    std::map<vertex_t, vertex_t>::iterator prev;
    vertex_t vertex = target;
    path.push_front(vertex);
    while((prev = previous.find(vertex)) != previous.end())
    {
        vertex = prev->second;
        path.push_front(vertex);
    }
    return path;
}

//****** Image Processing Functions ********//
double* pointSetBoundingRect( const Mat& points , Mat m){
    int npoints = points.checkVector(2);
	double return_array[4];

    int  xmin = 0, ymin = 0, xmax = -1, ymax = -1, i;
    Point ptxmin , ptymin , ptxmax , ptymax;

    if( npoints == 0 )
        return return_array;

    const Point* pts = points.ptr<Point>();
    Point pt = pts[0];

    ptxmin = ptymin = ptxmax = ptymax = pt;
    xmin = xmax = pt.x;
    ymin = ymax = pt.y;

    for( i = 1; i < npoints; i++ )
    {
        pt = pts[i];

        if( xmin > pt.x )
        {
            xmin = pt.x;
            ptxmin = pt;
        }


        if( xmax < pt.x )
        {
            xmax = pt.x;
            ptxmax = pt;
        }

        if( ymin > pt.y )
        {
            ymin = pt.y;
            ptymin = pt;
        }

        if( ymax < pt.y )
        {
            ymax = pt.y;
            ptymax = pt;
        }
    }
    ellipse( m, ptxmin, Size( 3, 3), 0, 0, 360, Scalar( 255, 0, 255 ), 2, 8, 0 );
    ellipse( m, ptxmax, Size( 3, 3), 0, 0, 360, Scalar( 255, 0, 255 ), 2, 8, 0 );
    ellipse( m, ptymin, Size( 3, 3), 0, 0, 360, Scalar( 255, 0, 255 ), 2, 8, 0 );
    ellipse( m, ptymax, Size( 3, 3), 0, 0, 360, Scalar( 255, 0, 255 ), 2, 8, 0 );
    return_array[0]=ptxmin.x;
    return_array[1]=ptxmin.y;
    return_array[2]=ptxmax.x;
    return_array[3]=ptxmax.y;
     
    return return_array;
}

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
    image_sub_ = it_.subscribe("/ardrone/image_raw", 1, 
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
class ImageConverter_Front{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter_Front()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/image_raw", 1, 
      &ImageConverter_Front::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video2", 1);

    cv::namedWindow("Front Camera");
  }

  ~ImageConverter_Front()
  {
    cv::destroyWindow("Front Camera");
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
	cv::Mat  temp;
	cv::Mat  hsv;
	cv::Mat  yellowThreshed;
	cv::Mat turquoiseThreshed;
	cv::Moments moment;
	double q;
	double focal=1097.51;
	cv::Scalar lower_yellow = cvScalar(45, 100, 100);
	cv::Scalar upper_yellow = cvScalar(75, 255, 255);
	cv::Scalar lower_turquoise = cvScalar(10, 170, 150);
	cv::Scalar upper_turquoise = cvScalar(25, 255, 255);
	//*****  Threshold Image    ********** //
	cvtColor(cv_ptr->image, hsv , CV_BGR2HSV);
	inRange(hsv, lower_yellow, upper_yellow, yellowThreshed);
	inRange(hsv, lower_turquoise, upper_turquoise, turquoiseThreshed);
	mean_yellow = mean(yellowThreshed);
	mean_turquoise = mean(turquoiseThreshed);
	if(mean_yellow[0] > 0.1){		
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
		nyc_turquoise= (moment.m01/moment.m00)-(yellowThreshed.rows/2);
		nxc_turquoise = (moment.m10/moment.m00)-(yellowThreshed.cols/2);
		
		//*****  Calculate Bearing Angles    ********** //
		q= nxc_turquoise/focal;
		bearing_turquoise=atan(q)*((180.0)/(3.14159));
		
		//*****  Draw Centers and Edges on Image    ******* //
		circle(cv_ptr->image, Point(int(nxc_turquoise+(turquoiseThreshed.cols/2)),int(nyc_turquoise+(turquoiseThreshed.rows/2))), 10, (180,0,255));
	}
    // Update GUI Window
    cv::imshow("Front Camera", cv_ptr->image);
    cv::waitKey(1);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
}; 
//******** Publisher Functions   ***********//
void set_velocity_uav(const double lx1, const double ly1, const double lz1, const double az1){
    geometry_msgs::Twist command1;
    command1.linear.x = lx1;
    command1.linear.y = ly1;
    command1.linear.z = lz1;
    command1.angular.z = az1;
    cmd_pub.publish(command1);
}
void takeoff(){
	std_msgs::Empty command1;
    takeoff_pub.publish(command1);
}

void land(){
	std_msgs::Empty command1;
    land_pub.publish(command1);
}

//******** Subscriber Callbacks  ***********//
void sonarCallback(const sensor_msgs::Range& msg){
    sonar_height = msg.range;
}   
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
void imuCallback(const nav_msgs::Odometry& msg){    
    uav_pose_x = msg.pose.pose.position.x;
    uav_pose_y = msg.pose.pose.position.y;
    double uav_orientation_x=msg.pose.pose.orientation.x;
    double uav_orientation_y=msg.pose.pose.orientation.y;
    double uav_orientation_z=msg.pose.pose.orientation.z;
    double uav_orientation_w=msg.pose.pose.orientation.w;
    
    double e1=uav_orientation_x;
    double e2=uav_orientation_y;
    double e0=uav_orientation_z;
    double e3=uav_orientation_w;
    
    uav_yaw = atan2(2*(e0*e3+e1*e2),(pow(e0,2)+pow(e1,2)-pow(e2,2)-pow(e3,2)));
    uav_yaw=uav_yaw*180/M_PI; 
    myfile2 << uav_pose_x << "," << uav_pose_y << "," << uav_yaw << endl;
}
//******** Other Functions   ************* //
void reset_communication(){
	ofstream myfile;
	double sentcheck = 0;
	myfile.open ("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/doorseen.txt");
	myfile << (double)sentcheck << endl;
	myfile.close();
}
void recieve_directions_ugv(){
	std::ifstream ifile("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/doorseen.txt", std::ios::in);
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
		door_seen = 1; // doorway is seen by ugv
	} 
	else{
		door_seen = 0; // doorway is not seen by ugv
	}
}
void is_passed(){
	double obstacle1_angle;
	double obstacle2_angle;
	double obstacle3_angle;
	
	obstacle1_angle = acos((pow(obstacle1_relative_distance,2)+pow(ugv_to_blue,2)-pow(ugv_relative_distance,2))/(2*obstacle1_relative_distance*ugv_to_blue));
	obstacle2_angle = acos((pow(obstacle2_relative_distance,2)+pow(ugv_to_green,2)-pow(ugv_relative_distance,2))/(2*obstacle2_relative_distance*ugv_to_green));
	obstacle3_angle = acos((pow(obstacle3_relative_distance,2)+pow(ugv_to_pink,2)-pow(ugv_relative_distance,2))/(2*obstacle3_relative_distance*ugv_to_pink));
	obstacle1_angle = obstacle1_angle * (180/M_PI);
	obstacle2_angle = obstacle2_angle * (180/M_PI);
	obstacle3_angle = obstacle3_angle * (180/M_PI);
	cout<< "Obstacle 1 Angle: " << obstacle1_angle << endl;
	cout<< "Obstacle 2 Angle: " << obstacle2_angle << endl;
	cout<< "Obstacle 3 Angle: " << obstacle3_angle << endl;
	if((obstacle1_angle < 90) && (mean_red[0] > 0.1) && (mean_blue[0] > 0.1) && (mean_orange[0] > 0.1) ){
		obstacle1_passed = 0;
	}
	else{
		obstacle1_passed = 1;
	}
	if((obstacle2_angle < 90) && (mean_red[0] > 0.1) && (mean_green[0] > 0.1) && (mean_orange[0] > 0.1)){
		obstacle2_passed = 0;
	}
	else{
		obstacle2_passed = 1;
	}

	if((obstacle3_angle < 90) && (mean_red[0] > 0.1) && (mean_pink[0] > 0.1) && (mean_orange[0] > 0.1)){
		obstacle3_passed = 0;
	}
	else{
		obstacle3_passed = 1;
	}
}
void send_path_to_ugv(){
	ofstream myfile;
	double sentcheck = 1;
	myfile.open ("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/communication.txt");
	myfile << (double)sentcheck << " " <<(double)obstacle1_direction << " " << (double)obstacle2_direction << " " << (double)obstacle3_direction << endl;
	myfile.close();
	myfile.open ("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/ispassed.txt");
	myfile << (double)obstacle1_passed << " " << (double)obstacle2_passed << " " << (double)obstacle3_passed << endl;
	myfile.close();
	myfile.open ("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/uavinroom.txt");
	myfile << (int)uav_in_room << endl;
	myfile.close();
}
void dijkstra_calculation(){
	//############# Dijkstras Stuff   ############# //
        adjacency_map_t adjacency_map;
        std::vector<std::string> vertex_names;
        vertex_names.push_back("Start");        // 0
		vertex_names.push_back("Blue Left");    // 1
		vertex_names.push_back("Blue Right");   // 2
		vertex_names.push_back("Green Left");   // 3
		vertex_names.push_back("Green Right");  // 4
		vertex_names.push_back("Pink Left");    // 5
		vertex_names.push_back("Pink Right");   // 6
		vertex_names.push_back("End");          // 7
		adjacency_map[0].push_back(edge(1,  ugv_right_obstacle1_left));
		adjacency_map[0].push_back(edge(2,  ugv_left_obstacle1_right));
		adjacency_map[1].push_back(edge(3,  obstacle1_left_obstacle2_left));
		adjacency_map[1].push_back(edge(4,  obstacle1_left_obstacle2_right));
		adjacency_map[2].push_back(edge(3,  obstacle1_right_obstacle2_left));
		adjacency_map[2].push_back(edge(4,  obstacle1_right_obstacle2_right));
		adjacency_map[3].push_back(edge(5,  obstacle2_left_obstacle3_left));
		adjacency_map[3].push_back(edge(6,  obstacle2_left_obstacle3_right));
		adjacency_map[4].push_back(edge(5,  obstacle2_right_obstacle3_left));
		adjacency_map[4].push_back(edge(6,  obstacle2_right_obstacle3_right));
		adjacency_map[5].push_back(edge(7,  obstacle3_left_end));
		adjacency_map[6].push_back(edge(7,  obstacle3_right_end));
		
		std::map<vertex_t, weight_t> min_distance;
		std::map<vertex_t, vertex_t> previous;
		DijkstraComputePaths(0, adjacency_map, min_distance, previous);
		for (adjacency_map_t::iterator vertex_iter = adjacency_map.begin(); vertex_iter != adjacency_map.end(); vertex_iter++){
        vertex_t v = vertex_iter->first;
        std::list<vertex_t> path =
            DijkstraGetShortestPathTo(v, previous);
        std::list<vertex_t>::iterator path_iter = path.begin();
        //std::cout << "Path: ";
        string final_path;
        for( ; path_iter != path.end(); path_iter++){
			final_path =  final_path + to_string (*path_iter);
        }
        //cout << final_path << endl;
        if (final_path.find("1") != std::string::npos) {
			obstacle1_direction = 1;
		}
		if (final_path.find("2") != std::string::npos) {
			obstacle1_direction = -1;
		}
		if (final_path.find("3") != std::string::npos) {
			obstacle2_direction = 1;
		}
		if (final_path.find("4") != std::string::npos) {
			obstacle2_direction = -1;
		}
		if (final_path.find("5") != std::string::npos) {
			obstacle3_direction = 1;
		}
		if (final_path.find("6") != std::string::npos) {
			obstacle3_direction = -1;
		}							
    }
}
void calculate_distances(){
   ugv_right_obstacle1_left = hypot((ugv_rightmost.x - blue_leftmost.x),( ugv_rightmost.y - blue_leftmost.y) );

   ugv_right_obstacle1_right = hypot(ugv_rightmost.x - blue_rightmost.x , ugv_rightmost.y - blue_rightmost.y ) ;
  
   ugv_left_obstacle1_left = hypot(ugv_leftmost.x - blue_leftmost.x, ugv_leftmost.y - blue_leftmost.y);  

   ugv_left_obstacle1_right = hypot(ugv_leftmost.x - blue_rightmost.x, ugv_leftmost.y - blue_rightmost.y);  
  
   obstacle1_left_obstacle2_left = hypot(blue_leftmost.x - green_leftmost.x, blue_leftmost.y - green_leftmost.y);
  
   obstacle1_left_obstacle2_right = hypot(blue_leftmost.x - green_rightmost.x, blue_leftmost.y - green_rightmost.y); 
  
   obstacle1_right_obstacle2_left  = hypot(blue_rightmost.x - green_leftmost.x, blue_rightmost.y - green_leftmost.y); 
 
   obstacle1_right_obstacle2_right = hypot(blue_rightmost.x - green_rightmost.x, blue_rightmost.y - green_rightmost.y); 
 
   obstacle2_left_obstacle3_left = hypot(green_leftmost.x - pink_leftmost.x, green_leftmost.y - pink_leftmost.y);   

   obstacle2_left_obstacle3_right = hypot(green_leftmost.x - pink_rightmost.x, green_leftmost.y - pink_rightmost.y);  
 
   obstacle2_right_obstacle3_left = hypot(green_rightmost.x - pink_leftmost.x, green_rightmost.y - pink_leftmost.y) ;  

   obstacle2_right_obstacle3_right = hypot(green_rightmost.x - pink_rightmost.x, green_rightmost.y - pink_rightmost.y);
 
   obstacle3_right_end = hypot(pink_rightmost.x - xc_orange, pink_rightmost.y-yc_orange);

   obstacle3_left_end = hypot(pink_leftmost.x - xc_orange, pink_leftmost.y-yc_orange);

   obstacle1_relative_distance= hypot(xc_blue-xc_orange, yc_blue-yc_orange); 

   obstacle2_relative_distance= hypot(xc_green-xc_orange, yc_green-yc_orange); 

   obstacle3_relative_distance= hypot(xc_pink-xc_orange, yc_pink-yc_orange); 

   ugv_relative_distance= hypot(xc_red-xc_orange, yc_red-yc_orange); 

   ugv_to_blue = hypot(xc_red - xc_blue, yc_red - yc_blue);

   ugv_to_green = hypot(xc_red - xc_green, yc_red- yc_green);

   ugv_to_pink = hypot(xc_red - xc_pink, yc_red - yc_pink);      
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
void track_ugv(double time){ 
	ros::Rate loop_rate(100);	
	ros::spinOnce();
	loop_rate.sleep(); 
    int q;
    for(q=0; q<time; q++){
		
		door_seen = 1;
		uav_in_room = 0;
		if((door_seen == 0) || (door_seen == 1 && uav_in_room ==1)){
			set_velocity_uav(0, 0, 0, 0);
		}
		else if (door_seen == 1 && uav_in_room==0){
				//set_velocity_uav(0,0,-.5,0);
				loop_rate.sleep(); 
				ros::spinOnce();
				double Kpr=.001;
				double Kir=0;
				double Kdr=0.0001;
				double dt = 0.01;
				
				double error_r;
				double set_r = 0;
				double integral_r = 0;
				double derivative_r;
				double prev_err_r = 0;
				double PD_roll;
				
				double direction_door;
				double rotate_uav;
				while(mean_yellow[0] > 1 || mean_turquoise[0] > 1){
					if(mean_yellow[0] > 1 && mean_turquoise[0] > 1){
						// ############# Roll PID      ##################### 						
						error_r = set_r - ((nxc_yellow+nxc_turquoise)/2);
						integral_r = integral_r + error_r*dt;
						derivative_r = (error_r - prev_err_r)/dt;
						PD_roll = Kpr*error_r + Kir*integral_r + Kdr*derivative_r;
						prev_err_r = error_r;
						cout << "Roll: " << PD_roll << endl;
						set_velocity_uav(.05,0,0,PD_roll);
					}
					else if (mean_yellow[0] > 1 && mean_turquoise[0] < 1){
						set_velocity_uav(.1,0,0,.15);
						cout << "Roll: .15" << endl;
					}
					else if (mean_yellow[0] < 1 && mean_turquoise[0] > 1){
						set_velocity_uav(.1,0,0,-.15);
						cout << "Roll: -.15" << endl;
					}
					loop_rate.sleep(); 
					ros::spinOnce();
				}
				set_velocity_uav(.1,0,0,0);
				sleep(1);
				set_velocity_uav(0,0,0,0);
				uav_in_room = 1;
				send_path_to_ugv();
			}
		loop_rate.sleep(); 
		ros::spinOnce();
	}
}
   
int main(int argc, char **argv){
	ros::init(argc, argv, "uav");
	ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ImageConverter_Front ic;
    //ImageConverter_Front ic_front;
    
    myfile2.open ("/home/jonlwowski/catkin_ws/src/cooperative_obstacle_avoidance/src/mav_path.csv");
    
    //   Publishers
    cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 100);
    land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 100);
    //  Subscribers
   // sonar_sub = n.subscribe("/ardrone/sonar_height", 100, sonarCallback);
    odom_sub=n.subscribe("/p2os/pose", 100, odomCallback);
    imu_sub = n.subscribe("/ardrone/odometry", 100, imuCallback);
    
    
    // Main Code
    while (ros::ok())
    { 
		takeoff();
		sleep(1);
        set_velocity_uav(0, 0, 0, 0);
        sleep(2);
        
        track_ugv(10000000);
        
        
        
        loop_rate.sleep();    
        ros::spinOnce(); 
        
        break;
    }
    
    myfile2.close();
    return 0;
    
}