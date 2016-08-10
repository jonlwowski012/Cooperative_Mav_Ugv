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
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


#include <iostream>
#include <fstream>
#include <vector>

// Definitions
#define ERROR_BOUND	0.0005


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
ros::Subscriber cam_info_sub;

//******* Global Variables   **********//
double ugv_pose_x;
double ugv_pose_y;
double ugv_yaw;
double direction;
double door_seen = 0;
int objects_detected = 0;
int uav_in_room = 0;
cv::Scalar mean_blue;
cv::Scalar mean_green;
cv::Scalar mean_pink;
cv::Scalar mean_yellow;
cv::Scalar mean_turquoise;
cv::Scalar mean_red;
Mat homography_stat;
Mat homography_move;
Point Red_center;
Point Blue_center;
Point Green_center[3];
Point Pink_center[3];
Point Red_center_original = Point(0,0);
Point Blue_center_original = Point(0,0);
Point Green_center_original[3] = {Point(0,0),Point(0,0),Point(0,0)};
Point Pink_center_original[3]  = {Point(0,0),Point(0,0),Point(0,0)};
geometry_msgs::Pose marker_pose[10];
boost::array<double, 9> calibration;
int flag = 0;


//****** Image Processing Functions ********//
double Distance(double dX0, double dY0, double dX1, double dY1){
    return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}
void cameraPoseFromHomography(const Mat& H, Mat& pose){
	
    pose = Mat::eye(3, 4, CV_32FC1);      // 3x4 matrix, the camera pose
    float norm1 = (float)norm(H.col(0));  
    float norm2 = (float)norm(H.col(1));  
    float tnorm = (norm1 + norm2) / 2.0f; // Normalization value

    cv::normalize(H.col(0), pose.col(0));   // Normalize the rotation, and copies the column to pose
    cv::normalize(H.col(1), pose.col(1));   // Normalize the rotation and copies the column to pose

    Mat p3 = pose.col(0).cross(pose.col(1));   // Computes the cross-product of p1 and p2
    Mat c2 = pose.col(2);    // Pointer to third column of pose
    (pose.col(0).cross(pose.col(1))).copyTo(pose.col(2));       // Third column is the crossproduct of columns one and two

    pose.col(3) = norm(H.col(2) / tnorm);  //vector t [R|t] is the last column of pose
    double phi_x = atan2(pose.at<float>(2,1),pose.at<float>(2,2));
	double phi_y = atan2(-(pose.at<float>(2,0)), sqrt((pose.at<float>(2,1)*pose.at<float>(2,1)) + (pose.at<float>(2,2)*pose.at<float>(2,2))));
	double phi_z = atan2(pose.at<float>(1,0),pose.at<float>(0,0));
	cout<< "rot x: " << phi_x << endl << " rot y: " << phi_y << endl << " rot z: " << phi_z << endl;
    //cout << "pose: " << pose << endl;
        
}
void getOriginalPoints(){
	Red_center_original = Red_center;
	Blue_center_original = Blue_center;
	for(int i = 0 ; i <4 ; i++){
		Green_center_original[i]=Green_center[i];
	}
	for(int i = 0 ; i <4 ; i++){
	   Pink_center_original[i] = Pink_center[i];
   }
}
void reorder_points(Point arr[], Point single, Point ret_array[]){
	//**** ret_array = [Single, Diagonal, Negative Cross, Positive Cross] *****//
	
	ret_array[0] = single;
	double vector1 = Distance(single.x, single.y, arr[0].x, arr[0].y);
	double vector2 = Distance(single.x, single.y, arr[1].x, arr[1].y);
	double vector3 = Distance(single.x, single.y, arr[2].x, arr[2].y);
	
	//***** Calculate Longest Vector ***************//
	if(vector1 > vector2 && vector1 > vector3){
		ret_array[1] = arr[0];
	}
	if(vector2 > vector1 && vector2 > vector3){
		ret_array[1] = arr[1];
	}
	if(vector3 > vector2 && vector3 > vector1){
		ret_array[1] = arr[2];
	}
	
	//***** Calculate Cross Products   *******//
	Point longvector;
	Point shortvector1;
	Point shortvector2;
	Point shortvector3;
	longvector.x = ret_array[1].x - single.x;
	longvector.y = ret_array[1].y - single.y;
	shortvector1.x = arr[0].x - single.x;
	shortvector1.y = arr[0].y - single.y;
	shortvector2.x = arr[1].x - single.x;
	shortvector2.y = arr[1].y - single.y;
	shortvector3.x = arr[2].x - single.x;
	shortvector3.y = arr[2].y - single.y;
	
	double vector1cross = longvector.cross(shortvector1);
	double vector2cross = longvector.cross(shortvector2);
	double vector3cross = longvector.cross(shortvector3);
	
	// ******* Fill in array with values ****////
	if(vector1cross < 0){
		ret_array[2] = arr[0];
	}
	else if (vector2cross < 0){
		ret_array[2] = arr[1];
	}
	else if (vector3cross<0){
		ret_array[2] = arr[2];
	}
	
	if(vector1cross > 0){
		ret_array[3] = arr[0];
	}
	else if (vector2cross > 0){
		ret_array[3] = arr[1];
	}
	else if (vector3cross > 0){
		ret_array[3] = arr[2];
	}
	
}

int homographyDecompose(Mat& hom){
	double sign;
	double d_dash;
	double st, ct; // sin(theta) and cos(theta). 'float' type to 
				  // avoid numerical errors
	double x1, x3;
	double scaleFactor;	// the 'd' in decomposition alg. p 290 faugeras
	int flag_AllEigEqual = 0;
	Mat dh_u = Mat(3,3, CV_64F);
	Mat dh_v = Mat(3,3, CV_64F);
	Mat dh_work = Mat(3,1, CV_64F);
	Mat dh_s = Mat(3,1, CV_64F);
	Mat m_TransV = Mat(3,3, CV_64F);
	Mat m_nstarActual = Mat(3,1, CV_64F);
	Mat m_N1 = Mat(3,1, CV_64F);
	Mat m_N2 = Mat(3,1, CV_64F);
	Mat m_N3 = Mat(3,1, CV_64F);
	Mat m_N4 = Mat(3,1, CV_64F);
	Mat m_R_dash = Mat(3,3, CV_64F);
	Mat m_R1 = Mat(3,3, CV_64F);
	Mat m_Hn = Mat(3,3, CV_64F);
	Mat m_R2 = Mat(3,3, CV_64F);
	Mat m_R3 = Mat(3,3, CV_64F);
	Mat m_R4 = Mat(3,3, CV_64F);
	Mat m_Rn = Mat(3,4, CV_64F);
	Mat m_Rf = Mat(3,4, CV_64F);
	Mat m_x_dash = Mat(3,1, CV_64F);
	Mat m_T1 = Mat(3,1, CV_64F);
	Mat m_T2 = Mat(3,1, CV_64F);
	Mat m_T3 = Mat(3,1, CV_64F);
	Mat m_T4 = Mat(3,1, CV_64F);
	Mat m_SolutionSet = Mat(4,1, CV_64F);
	Mat m_R = Mat(3,3, CV_64F);
	Mat m_T = Mat(3,1, CV_64F);
	Mat m_N = Mat(3,1, CV_64F);
	Mat m_alpha_g33 = Mat(4,1, CV_64F);
	Mat m_alpha = Mat(4,1, CV_64F);
	Mat m_CiHCalibrationMatrix = Mat(3,3, CV_64F);
	Mat m_InvCiHCalibrationMatrix = Mat(3,3, CV_64F);
	double m_U_Deter, m_V_Deter;
	
	//Fill in Camera Calibration Matrix
	m_CiHCalibrationMatrix.at<double>(0,0) = calibration[0];
	m_CiHCalibrationMatrix.at<double>(0,1) = calibration[1];
	m_CiHCalibrationMatrix.at<double>(0,2) = calibration[2];
	m_CiHCalibrationMatrix.at<double>(1,0) = calibration[3];
	m_CiHCalibrationMatrix.at<double>(1,1) = calibration[4];
	m_CiHCalibrationMatrix.at<double>(1,2) = calibration[5];
	m_CiHCalibrationMatrix.at<double>(2,0) = calibration[6];
	m_CiHCalibrationMatrix.at<double>(2,1) = calibration[7];
	m_CiHCalibrationMatrix.at<double>(2,2) = calibration[8];
	
	//cout << "Cal: "<< endl << m_CiHCalibrationMatrix << endl;
	// Calculate Inverse of Camera Calibration Matrix
	invert(m_CiHCalibrationMatrix, m_InvCiHCalibrationMatrix);
	//m_Hn calculation
	m_Hn = m_InvCiHCalibrationMatrix * (hom * m_CiHCalibrationMatrix);

	//SVD of Homography Matrix
	cv::SVD::compute(m_Hn, dh_s, dh_u, dh_v);
	//cout << "dh_s " << endl << dh_s << endl;
	transpose(dh_v, dh_v);
	
	// compute alpha.G33 for each image correspondance
/*	for (int i = 0; i < numPoints; i++){
		m_alpha_g33.at<double>(i,0) = 1.0/(m_Gn.at<double>(2,0) * m_pi_star.at<double>(0,i) 
									+ m_Gn.at<double>(2,1) * m_pi_star.at<double>(1,i) + 1.0);
	}*/
										
	
	
	//Transpose of V
	transpose(dh_v, m_TransV);
	   

	// s = det(U) * det(V)
	m_U_Deter = determinant(dh_u);
	m_V_Deter = determinant(dh_v);
	sign = m_U_Deter * m_V_Deter;
	
	// Based on the sign of d_dash, find R_dash and x_dash
	// NOTE: Using error bounds instead. Is 0.9999 = 1.0001?
	
	if( (fabs(dh_s.at<double>(0,0) - dh_s.at<double>(1,0)) > ERROR_BOUND) && (fabs(dh_s.at<double>(1,0) - dh_s.at<double>(2,0)) > ERROR_BOUND) )
	{
				cout<< "Eigenvalues are DISTINCT" << endl;

		x1 = sqrt((dh_s.at<double>(0,0)*dh_s.at<double>(0,0) - dh_s.at<double>(1,0)*dh_s.at<double>(1,0))/(dh_s.at<double>(0,0)*dh_s.at<double>(0,0) - dh_s.at<double>(2,0)*dh_s.at<double>(2,0)));
		x3 = sqrt((dh_s.at<double>(1,0)*dh_s.at<double>(1,0) - dh_s.at<double>(2,0)*dh_s.at<double>(2,0))/(dh_s.at<double>(0,0)*dh_s.at<double>(0,0) - dh_s.at<double>(2,0)*dh_s.at<double>(2,0)));

		st = sqrt((dh_s.at<double>(0,0)*dh_s.at<double>(0,0) - dh_s.at<double>(1,0)*dh_s.at<double>(1,0))*(dh_s.at<double>(1,0)*dh_s.at<double>(1,0) - dh_s.at<double>(2,0)*dh_s.at<double>(2,0)))/(dh_s.at<double>(1,0) * (dh_s.at<double>(0,0) + dh_s.at<double>(2,0)));
		ct = (dh_s.at<double>(1,0) * dh_s.at<double>(1,0) + dh_s.at<double>(0,0) * dh_s.at<double>(2,0))/(dh_s.at<double>(1,0) * (dh_s.at<double>(0,0) + dh_s.at<double>(2,0)));		
		
		// d
		scaleFactor = sign*(dh_s.at<double>(1,0));
		
		// --------------------------------- Solution # 1
		// np
		m_nstarActual.at<double>(0,0) = x1;
		m_nstarActual.at<double>(1,0) = 0;
		m_nstarActual.at<double>(2,0) = x3;
		
		// N
		m_N1 = dh_v*m_nstarActual;
		
		// Rp
		m_R_dash.at<double>(0,0) = ct;
		m_R_dash.at<double>(0,1) = 0;
		m_R_dash.at<double>(0,2) = -1*st;
		m_R_dash.at<double>(1,0) = 0;
		m_R_dash.at<double>(1,1) = 1;
		m_R_dash.at<double>(1,2) = 0;
		m_R_dash.at<double>(2,0) = st;
		m_R_dash.at<double>(2,1) = 0;
		m_R_dash.at<double>(2,2) = ct;

		m_R1 = (dh_u * m_R_dash) * m_TransV;
		m_R1 = m_R1 * sign;
		
		// Rn
		m_Rn.at<double>(2,1) = m_R1.at<double>(2,0)*m_N1.at<double>(0,0) + m_R1.at<double>(2,1)*m_N1.at<double>(1,0) + m_R1.at<double>(2,2)*m_N1.at<double>(2,0);
		m_Rf.at<double>(2,1) = m_R1.at<double>(0,2)*m_N1.at<double>(0,0) + m_R1.at<double>(1,2)*m_N1.at<double>(1,0) + m_R1.at<double>(2,2)*m_N1.at<double>(2,0);

		// tp
		m_x_dash.at<double>(0,0) = (dh_s.at<double>(0,0) - dh_s.at<double>(2,0)) * x1;
		m_x_dash.at<double>(1,0) = 0;
		m_x_dash.at<double>(2,0) = -1 * (dh_s.at<double>(0,0) - dh_s.at<double>(2,0)) * x3;
		
		// T
		m_T1 = dh_u * m_x_dash;
		
		// --------------------------------- Solution # 2
		// np
		m_nstarActual.at<double>(0,0) = -1*x1;
		m_nstarActual.at<double>(1,0) = 0;
		m_nstarActual.at<double>(2,0) = x3;

		// N
		m_N2 = dh_v*m_nstarActual;

		// Rp
		m_R_dash.at<double>(0,0) = ct;
		m_R_dash.at<double>(0,1) = 0;
		m_R_dash.at<double>(0,2) = st;
		m_R_dash.at<double>(1,0) = 0;
		m_R_dash.at<double>(1,1) = 1;
		m_R_dash.at<double>(1,2) = 0;
		m_R_dash.at<double>(2,0) = -1*st;
		m_R_dash.at<double>(2,1) = 0;
		m_R_dash.at<double>(2,2) = ct;

		m_R2 = (dh_u * m_R_dash) * m_TransV;
		m_R2 = m_R2 * sign;
		
		// Rn
		m_Rn.at<double>(2,1) = m_R2.at<double>(2,0)*m_N2.at<double>(0,0) + m_R2.at<double>(2,1)*m_N2.at<double>(1,0) + m_R2.at<double>(2,2)*m_N2.at<double>(2,0);
		m_Rf.at<double>(2,1) = m_R2.at<double>(0,2)*m_N2.at<double>(0,0) + m_R2.at<double>(1,2)*m_N2.at<double>(1,0) + m_R2.at<double>(2,2)*m_N2.at<double>(2,0);

		// tp
		m_x_dash.at<double>(0,0) = -1 * (dh_s.at<double>(0,0) - dh_s.at<double>(2,0)) * x1;
		m_x_dash.at<double>(1,0) = 0;
		m_x_dash.at<double>(2,0) = -1 * (dh_s.at<double>(0,0) - dh_s.at<double>(2,0)) * x3;

		// T
		m_T2 = dh_u * m_x_dash;
		
		// --------------------------------- Solution # 3
		// np
		m_nstarActual.at<double>(0,0) = x1;
		m_nstarActual.at<double>(1,0) = 0;
		m_nstarActual.at<double>(2,0) = -1*x3;

		// N
		m_N3 = dh_v*m_nstarActual;

		// Rp
		m_R_dash.at<double>(0,0) = ct;
		m_R_dash.at<double>(0,1) = 0;
		m_R_dash.at<double>(0,2) = st;
		m_R_dash.at<double>(1,0) = 0;
		m_R_dash.at<double>(1,1) = 1;
		m_R_dash.at<double>(1,2) = 0;
		m_R_dash.at<double>(2,0) = -1*st;
		m_R_dash.at<double>(2,1) = 0;
		m_R_dash.at<double>(2,2) = ct;

		m_R3 = (dh_u * m_R_dash) * m_TransV;
		m_R3 = m_R3 * sign;
		
		// Rn
		m_Rn.at<double>(2,2) = m_R3.at<double>(2,0)*m_N3.at<double>(0,0) + m_R3.at<double>(2,1)*m_N3.at<double>(1,0) + m_R3.at<double>(2,2)*m_N3.at<double>(2,0);
		m_Rf.at<double>(2,2) = m_R3.at<double>(0,2)*m_N3.at<double>(0,0) + m_R3.at<double>(1,2)*m_N3.at<double>(1,0) + m_R3.at<double>(2,2)*m_N3.at<double>(2,0);

		// tp
		m_x_dash.at<double>(0,0) = (dh_s.at<double>(0,0) - dh_s.at<double>(2,0)) * x1;
		m_x_dash.at<double>(1,0) = 0;
		m_x_dash.at<double>(2,0) = (dh_s.at<double>(0,0) - dh_s.at<double>(2,0)) * x3;

		// T
		m_T3 = dh_u * m_x_dash;
		
		// --------------------------------- Solution # 4
		// np
		m_nstarActual.at<double>(0,0) = -1*x1;
		m_nstarActual.at<double>(1,0) = 0;
		m_nstarActual.at<double>(2,0) = -1*x3;

		// N
		m_N4 = dh_v*m_nstarActual;

		// Rp
		m_R_dash.at<double>(0,0) = ct;
		m_R_dash.at<double>(0,1) = 0;
		m_R_dash.at<double>(0,2) = -1*st;
		m_R_dash.at<double>(1,0) = 0;
		m_R_dash.at<double>(1,1) = 1;
		m_R_dash.at<double>(1,2) = 0;
		m_R_dash.at<double>(2,0) = st;
		m_R_dash.at<double>(2,1) = 0;
		m_R_dash.at<double>(2,2) = ct;

		m_R4 = (dh_u * m_R_dash) * m_TransV;
		m_R4 = m_R4 * sign;
		
		// Rn
		m_Rn.at<double>(2,3) = m_R4.at<double>(2,0)*m_N4.at<double>(0,0) + m_R4.at<double>(2,1)*m_N4.at<double>(1,0) + m_R4.at<double>(2,2)*m_N4.at<double>(2,0);
		m_Rf.at<double>(2,3) = m_R4.at<double>(0,2)*m_N4.at<double>(0,0) + m_R4.at<double>(1,2)*m_N4.at<double>(1,0) + m_R4.at<double>(2,2)*m_N4.at<double>(2,0);

		// tp
		m_x_dash.at<double>(0,0) = -1 * (dh_s.at<double>(0,0) - dh_s.at<double>(2,0)) * x1;
		m_x_dash.at<double>(1,0) = 0;
		m_x_dash.at<double>(2,0) = (dh_s.at<double>(0,0) - dh_s.at<double>(2,0)) * x3;

		// T
		m_T4 = dh_u * m_x_dash;
		
		// Reduce solutions - features in front of the camera
		int i,j = 0;
		for(i=0; i<4; i++)
		{
			if (m_Rn.at<double>(2,i) > 0)
			{	
				m_SolutionSet.at<double>(j,0) = i;
				j = j + 1;
			}				
		}
		// Chose solution by nz
		if (j == 1)	
		{
			i = m_SolutionSet.at<double>(j,0);
			if (i == 0)
			{
				m_R = m_R1;
				m_T = m_T1;
				m_N = m_N1;
			}
			else if(i == 1)
			{
				m_R = m_R2;
				m_T = m_T2;
				m_N = m_N2;	
			}
			else if(i == 2)
			{
				m_R = m_R3;
				m_T = m_T3;
				m_N = m_N3;
			}
			else if(i == 3)
			{
				m_R = m_R4;
				m_T = m_T4;
				m_N = m_N4;
			}
		}
		else
		{
			int i1 = m_SolutionSet.at<double>(0,0);
			int i2 = m_SolutionSet.at<double>(1,0);
			if (m_Rn.at<double>(2,i1) > m_Rn.at<double>(2,i2))
			{
				if (i1 == 0)
				{
					m_R = m_R1;
					m_T = m_T1;
					m_N = m_N1;
				}
				else if(i1 == 1)
				{
					m_R = m_R2;
					m_T = m_T2;
					m_N = m_N2;	
				}
				else if(i1 == 2)
				{
					m_R = m_R3;
					m_T = m_T3;
					m_N = m_N3;
				}
				else if(i1 == 3)
				{
					m_R = m_R4;
					m_T = m_T4;
					m_N = m_N4;
				}
			}
			else
			{
				if (i2 == 0)
				{
					m_R = m_R1;
					m_T = m_T1;
					m_N = m_N1;
				}
				else if(i2 == 1)
				{
					m_R = m_R2;
					m_T = m_T2;
					m_N = m_N2;	
				}
				else if(i2 == 2)
				{
					m_R = m_R3;
					m_T = m_T3;
					m_N = m_N3;
				}
				else if(i2 == 3)
				{
					m_R = m_R4;
					m_T = m_T4;
					m_N = m_N4;
				}
			}
		}
		
	}
	// all equal singular values (pure rotation, mostly)
	if( (fabs(dh_s.at<double>(0,0) - dh_s.at<double>(1,0)) <= ERROR_BOUND) && (fabs(dh_s.at<double>(1,0) - dh_s.at<double>(2,0)) <= ERROR_BOUND) )
	{		
		cout << "Eigenvalues are EQUAL" << endl;
		// d
		scaleFactor = 0;

		// R
		m_R = m_Hn;

		// T
		m_T.at<double>(0,0) = 0;
		m_T.at<double>(1,0) = 0;
		m_T.at<double>(2,0) = 0;

		// N
		m_N.at<double>(0,0) = 0;
		m_N.at<double>(1,0) = 0;
		m_N.at<double>(2,0) = 0;

		flag_AllEigEqual = 1;
	}
	// two equal singular values (translation is normal to the plane)
	if (flag_AllEigEqual == 0)
	{
		if( (fabs(dh_s.at<double>(0,0) - dh_s.at<double>(1,0)) <= ERROR_BOUND) || (fabs(dh_s.at<double>(1,0) - dh_s.at<double>(2,0)) <= ERROR_BOUND))
		{
			cout << "TWO eigenvalues are EQUAL" << endl;

			// d
			scaleFactor = sign*dh_s.at<double>(1,0);
			
			// --------------------------------- Solution # 1
			// np
			m_nstarActual.at<double>(0,0) = 0;
			m_nstarActual.at<double>(1,0) = 0;
			m_nstarActual.at<double>(2,0) = 1;

			// N
			m_N1 = dh_v*m_nstarActual;

			// Rp
			m_R_dash.at<double>(0,0) = 1;
			m_R_dash.at<double>(0,1) = 0;
			m_R_dash.at<double>(0,2) = 0;
			m_R_dash.at<double>(1,0) = 0;
			m_R_dash.at<double>(1,1) = 1;
			m_R_dash.at<double>(1,2) = 0;
			m_R_dash.at<double>(2,0) = 0;
			m_R_dash.at<double>(2,1) = 0;
			m_R_dash.at<double>(2,2) = 1;

			m_R1 = (dh_u * m_R_dash) * m_TransV;
			m_R1 = m_R1 * sign;
			
			// Rn & Rf
			m_Rn.at<double>(2,0) = m_R1.at<double>(2,0)*m_N1.at<double>(0,0) + m_R1.at<double>(2,1)*m_N1.at<double>(1,0) + m_R1.at<double>(2,2)*m_N1.at<double>(2,0);
			m_Rf.at<double>(2,0) = m_R1.at<double>(0,2)*m_N1.at<double>(0,0) + m_R1.at<double>(1,2)*m_N1.at<double>(1,0) + m_R1.at<double>(2,2)*m_N1.at<double>(2,0);

			// tp
			m_x_dash.at<double>(0,0) = 0;
			m_x_dash.at<double>(1,0) = 0;
			m_x_dash.at<double>(2,0) = (dh_s.at<double>(2,0) - dh_s.at<double>(0,0));

			// T
			m_T1 = dh_u * m_x_dash;

			// --------------------------------- Solution # 2
			// np
			m_nstarActual.at<double>(0,0) = 0;
			m_nstarActual.at<double>(1,0) = 0;
			m_nstarActual.at<double>(2,0) = -1;

			// N
			m_N2 = dh_v * m_nstarActual;

			// Rp
			m_R_dash.at<double>(0,0) = 1;
			m_R_dash.at<double>(0,1) = 0;
			m_R_dash.at<double>(0,2) = 0;
			m_R_dash.at<double>(1,0) = 0;
			m_R_dash.at<double>(1,1) = 1;
			m_R_dash.at<double>(1,2) = 0;
			m_R_dash.at<double>(2,0) = 0;
			m_R_dash.at<double>(2,1) = 0;
			m_R_dash.at<double>(2,2) = 1;

			m_R2 = (dh_u * m_R_dash) * m_TransV;
			m_R2 = m_R2 * sign;
			
			// Rn & Rf
			m_Rn.at<double>(2,1) = m_R2.at<double>(2,0)*m_N2.at<double>(0,0) + m_R2.at<double>(2,1)*m_N2.at<double>(1,0) + m_R2.at<double>(2,2)*m_N2.at<double>(2,0);
			m_Rf.at<double>(2,1) = m_R2.at<double>(0,2)*m_N2.at<double>(0,0) + m_R2.at<double>(1,2)*m_N2.at<double>(1,0) + m_R2.at<double>(2,2)*m_N2.at<double>(2,0);

			// tp
			m_x_dash.at<double>(0,0) = 0;
			m_x_dash.at<double>(1,0) = 0;
			m_x_dash.at<double>(2,0) = -1 * (dh_s.at<double>(2,0) - dh_s.at<double>(0,0));

			// T
			m_T2 = dh_u * m_x_dash;
			
			// Reduce solutions - features in front of the camera
			int i,j = 0;
			for(i=0; i<2; i++)
			{
				if (m_Rn.at<double>(2,i) > 0)
				{	
					m_SolutionSet.at<double>(j,0) = i;
					j = j + 1;
				}				
			}

			// Chose solution by nz
			if (j == 1)	
			{
				i = (int)m_SolutionSet.at<double>(j,0);
				if (i == 0)
				{
					m_R = m_R1;
					m_T = m_T1;
					m_N = m_N1;
				}
				else if(i == 1)
				{
					m_R = m_R2;
					m_T = m_T2;
					m_N = m_N2;	
				}
			}
			else
			{
				int i1 = m_SolutionSet.at<double>(0,0);
				int i2 = m_SolutionSet.at<double>(1,0);
				if (m_Rf.at<double>(2,i1) > m_Rf.at<double>(2,i2))
				{
					if (i1 == 0)
					{	
						m_R = m_R1;
						m_T = m_T1;
						m_N = m_N1;
					}
					else if(i1 == 1)
					{
						m_R = m_R2;
						m_T = m_T2;
						m_N = m_N2;	
					}
				}
				else
				{
					if (i2 == 1)
					{
						m_R = m_R1;
						m_T = m_T1;
						m_N = m_N1;
					}
					else if(i2 == 2)
					{
						m_R = m_R2;
						m_T = m_T2;
						m_N = m_N2;	
					}
				}
			}
		}
	}
	/*for(int index = 0; index < m_alpha_g33.GetNumElements(); index++){
		m_alpha.at<double>(index,0) = m_alpha_g33.at<double>(index,0) * scaleFactor;
	}*/
	double phi_x = atan2(m_R.at<double>(2,1),m_R.at<double>(2,2));
	double phi_y = atan2(-(m_R.at<double>(2,0)), sqrt((m_R.at<double>(2,1)*m_R.at<double>(2,1)) + (m_R.at<double>(2,2)*m_R.at<double>(2,2))));
	double phi_z = atan2(m_R.at<double>(1,0),m_R.at<double>(0,0));
	cout<< "rot x: " << phi_x << endl << " rot y: " << phi_y << endl << " rot z: " << phi_z << endl;
	return 0;
}
bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i < j );
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
	cv::Mat  hsv;
	cv::Mat  temp;
	cv::Mat redThreshed;
	cv::Mat  blueThreshed;
	cv::Mat  greenThreshed;
	cv::Mat  pinkThreshed;
	cv::Mat  yellowThreshed;
	cv::Mat turquoiseThreshed;
	cv::Moments moment;
	cv::Scalar lower_red = cvScalar(0, 100, 100);
	cv::Scalar upper_red = cvScalar(10, 255, 255);
	cv::Scalar lower_blue = cvScalar(110, 50, 50);
	cv::Scalar upper_blue = cvScalar(130, 255, 255);
	cv::Scalar lower_green = cvScalar(50, 30, 30);
	cv::Scalar upper_green = cvScalar(70, 255, 255);
	cv::Scalar lower_pink = cvScalar(143, 0, 0);
	cv::Scalar upper_pink = cvScalar(162, 255, 255);
	cv::Scalar lower_yellow = cvScalar(20, 0, 0);
	cv::Scalar upper_yellow = cvScalar(30, 255, 255);
	cv::Scalar lower_turquoise = cvScalar(90, 100, 100);
	cv::Scalar upper_turquoise = cvScalar(100, 255, 255);
	//*****  Threshold Image    ********** //
	
	cvtColor(cv_ptr->image, hsv , CV_BGR2HSV);
	//medianBlur(hsv, hsv, 5);
	inRange(hsv, lower_red, upper_red, redThreshed);
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
	mean_red = mean(redThreshed);
	   

	if(mean_blue[0] > 0.1){		
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		int largest_area=0;
		int largest_contour_index=0;
		//cv::imshow("Blue", blueThreshed);
		/// Find contours
		findContours( blueThreshed, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		sort(contours.begin(), contours.end(), compareContourAreas);
		
		/// Get the moments
		Moments mu;
		mu = moments( contours[contours.size()-1], false );

		///  Get the mass centers:
		Point2f mc;
		mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
		Blue_center = mc;
		//*****  Draw Center on Image    ******* //
		circle(cv_ptr->image, mc, 10, (180,0,255));
		//cout << "MC blue: " << mc << " Largest index: " << largest_contour_index << endl;
	}
	if(mean_red[0] > 0.1){		
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		int largest_area=0;
		int largest_contour_index=0;
		//cv::imshow("Display", redThreshed);
		/// Find contours
		findContours( redThreshed, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		sort(contours.begin(), contours.end(), compareContourAreas);
		
		/// Get the moments
		Moments mu;
		mu = moments( contours[contours.size()-1], false );

		///  Get the mass centers:
		Point2f mc;
		mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
		Red_center = mc;
		//cout << "MC red: " << mc << endl;
		//*****  Draw Center on Image    ******* //
		circle(cv_ptr->image, mc, 10, (180,0,255));
		
		
	}
	if(mean_green[0] > 0.1){		
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		int largest_area=0;
		int largest_contour_index=0;
		int second_largest_area=0;
		int second_largest_contour_index=0;
		int third_largest_area=0;
		int third_largest_contour_index=0;
		//cv::imshow("Green", greenThreshed);
		/// Find contours
		findContours( greenThreshed, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		sort(contours.begin(), contours.end(), compareContourAreas);
		
		/// Get the moments
		Moments mu1;
		Moments mu2;
		Moments mu3;
		mu1 = moments( contours[contours.size()-1], false );
		mu2 = moments( contours[contours.size()-2], false );
		mu3 = moments( contours[contours.size()-3], false );
		
		///  Get the mass centers:
		Point2f mc1;
		Point2f mc2;
		Point2f mc3;
		mc1 = Point2f( mu1.m10/mu1.m00 , mu1.m01/mu1.m00 );
		mc2 = Point2f( mu2.m10/mu2.m00 , mu2.m01/mu2.m00 );
		mc3 = Point2f( mu3.m10/mu3.m00 , mu3.m01/mu3.m00 );
		Green_center[0] = mc1;
		Green_center[1] = mc2;
		Green_center[2] = mc3;
		//cout << "MC green 1: " << mc1 << endl;
		//cout << "MC green 2: " << mc2 << endl;
		//cout << "MC green 3: " << mc3 << endl;
		//*****  Draw Center on Image    ******* //
		circle(cv_ptr->image, mc1, 10, (180,0,255));
		circle(cv_ptr->image, mc2, 10, (180,0,255));
		circle(cv_ptr->image, mc3, 10, (180,0,255));
	}	
    if(mean_pink[0] > 0.1){	
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		int largest_area=0;
		int largest_contour_index=0;
		int second_largest_area=0;
		int second_largest_contour_index=0;
		int third_largest_area=0;
		int third_largest_contour_index=0;
		//cv::imshow("Pink", pinkThreshed);
		/// Find contours
		findContours( pinkThreshed, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		sort(contours.begin(), contours.end(), compareContourAreas);
		
		/// Get the moments
		Moments mu1;
		Moments mu2;
		Moments mu3;
		mu1 = moments( contours[contours.size()-1], false );
		mu2 = moments( contours[contours.size()-2], false );
		mu3 = moments( contours[contours.size()-3], false );
		
		///  Get the mass centers:
		Point2f mc1;
		Point2f mc2;
		Point2f mc3;
		mc1 = Point2f( mu1.m10/mu1.m00 , mu1.m01/mu1.m00 );
		mc2 = Point2f( mu2.m10/mu2.m00 , mu2.m01/mu2.m00 );
		mc3 = Point2f( mu3.m10/mu3.m00 , mu3.m01/mu3.m00 );
		Pink_center[0] = mc1;
		Pink_center[1] = mc2;
		Pink_center[2] = mc3;
		/*cout << "MC Pink 1: " << mc1 << endl;
		cout << "MC Pink 2: " << mc2 << endl;
		cout << "MC Pink 3: " << mc3 << endl;*/
		//*****  Draw Center on Image    ******* //
		circle(cv_ptr->image, mc1, 10, (180,0,255));
		circle(cv_ptr->image, mc2, 10, (180,0,255));
		circle(cv_ptr->image, mc3, 10, (180,0,255));
	}	
	if(mean_yellow[0] > 0.1){
		door_seen = 1;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		/// Find contours
		findContours( yellowThreshed, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		/// Get the moments
		vector<Moments> mu(contours.size() );
		for( int i = 0; i < contours.size(); i++ )
			{ mu[i] = moments( contours[i], false ); }

		///  Get the mass centers:
		vector<Point2f> mc( contours.size() );
		for( int i = 0; i < contours.size(); i++ )
			{ mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); } 
	}
    if(mean_turquoise[0] > 0.1){		
		moment = moments(turquoiseThreshed);
	}
    
    // Homography
    Mat decomposed_stat;
    Mat decomposed_move;
    std::vector<Point2f> stat_obj_orig;
    std::vector<Point2f> stat_obj_curr;
    std::vector<Point2f> mov_obj_orig;
    std::vector<Point2f> mov_obj_curr;
    
    Point mov_orig[4];
    Point mov_curr[4];
    Point stat_orig[4];
    Point stat_curr[4];
    
    
    if(flag == 0)
       {
		 sleep(2);
		 getOriginalPoints();
		 cout << "Getting Original Points" << endl;
		 sleep(2);
		 flag = 1;
	   }
   
    reorder_points(Green_center_original,Red_center_original, mov_orig);
	reorder_points(Green_center,Red_center, mov_curr);
    reorder_points(Pink_center,Blue_center,stat_curr);
    reorder_points(Pink_center_original,Blue_center_original, stat_orig);
    
    stat_obj_orig.push_back(stat_orig[0]);
	stat_obj_orig.push_back(stat_orig[1]); 
	stat_obj_orig.push_back(stat_orig[2]); 
	stat_obj_orig.push_back(stat_orig[3]);
	mov_obj_orig.push_back(mov_orig[0]);
	mov_obj_orig.push_back(mov_orig[1]); 
	mov_obj_orig.push_back(mov_orig[2]); 
	mov_obj_orig.push_back(mov_orig[3]);   
    stat_obj_curr.push_back(stat_curr[0]);
    stat_obj_curr.push_back(stat_curr[1]); 
    stat_obj_curr.push_back(stat_curr[2]); 
    stat_obj_curr.push_back(stat_curr[3]);
    mov_obj_curr.push_back(mov_curr[0]);
    mov_obj_curr.push_back(mov_curr[1]); 
    mov_obj_curr.push_back(mov_curr[2]); 
    mov_obj_curr.push_back(mov_curr[3]); 
    
    homography_stat = findHomography( stat_obj_orig, stat_obj_curr );
    homography_move = findHomography( mov_obj_orig, mov_obj_curr );
    
    //cout<<"Homography: " << endl << homography_move << endl;
    //cout<<"Original: " << endl << mov_obj_orig << endl;
    //cout<<"Current: " << endl << mov_obj_curr << endl;
    
    //cameraPoseFromHomography(homography_stat,decomposed_stat);
    homographyDecompose(homography_stat);
    
    //cout<<"decomposed stat: " << endl <<  homography_stat << endl << " decomposed move: " << endl << homography_move  << endl;
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
void cam_info_Callback(const sensor_msgs::CameraInfo& msg){
    calibration = msg.K;
}

//******** Other Functions   ************* //

int main(int argc, char **argv){
	ros::init(argc, argv, "ugv");
	ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ImageConverter ic;

    //   Publishers
    cmd_pub = n.advertise<geometry_msgs::Twist>("/p3dx/cmd_vel", 100);
    
    //  Subscribers
    cam_info_sub=n.subscribe("/p3dx/front_camera/camera_info", 100, cam_info_Callback);
    odom_sub=n.subscribe("/p3dx/odom", 100, odomCallback);
    
    /********* Setting Up Deleter and Spawner   *************************/
	deleter = n.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
	spawner = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    
    // Main Code
    
	

    while (ros::ok())
    { 
		
        loop_rate.sleep();
        ros::spinOnce();
        
    }

    
    
    return 0;
}
