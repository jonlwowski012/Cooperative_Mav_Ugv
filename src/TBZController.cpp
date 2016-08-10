// **********************************************************************************
// Program Written by Siddhartha Satish Mehta
// Email: siddhart@ufl.edu
// Teach by Zooming Visual Servo Control (TBZ)
// Last Modified on 11/18/2005
// **********************************************************************************

#pragma once
using namespace std;

// Header Files
#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <iostream>
#include "TBZController.h"
#include "LinearAlgebra.h"
#include "MainRobot.h"
#include "Robotics.h"
#include "cv.h"
#include "highgui.h"
#include <gsl/gsl_linalg.h>
#include <time.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <string.h>

// Disable Warnings
#pragma warning ( disable : 4244 )
#pragma warning ( disable : 4288 )
#pragma warning ( disable : 4551 )

// Global Variables
static gsl_vector *gh_work;
static gsl_matrix *gh_u;
static gsl_vector *gh_s;
static gsl_matrix *gh_v;
static gsl_vector *gh_b;
static gsl_vector *gh_x;

static gsl_vector *dh_work;
static gsl_matrix *dh_u;
static gsl_vector *dh_s;
static gsl_matrix *dh_v;

int flagFirstFrame = 1;
int flagEqualEigenValues = 0;

// Definitions
#define ERROR_BOUND	0.005

extern MainRobot Robot;
extern Robotics	C3;

// **********************************************************************************
// TBZ Main Controller
// **********************************************************************************

void TBZController::TBZControlMain(CvPoint2D32f *points[2], CvPoint2D32f *DesiredPts[2], int frame_count)
{
	// Controller 
	double ts;
	double temp;
	zstar_hat_dot = 0;

	struct _timeb tstruct;
	time_t ltime;
	time( &ltime );
	_ftime( &tstruct );	
	int time_ms = tstruct.millitm + (tstruct.time & 0xfffff) * 1000;
	if (frame_count == 1)
		time_ms_OLD = time_ms;
	
	fprintf(fTime,"%d\n",time_ms);	

	// Allocate memory		
	m_zhat.MatZeros(3,1);
	m_zhat.m_Value[2][0] = 1;

	// Obtain current and desired pixel coordinates
	TBZTargetCoord(points, DesiredPts, frame_count);	

	// Solve homography
	TBZHomography(frame_count);

	// u and theta
	m_INTCALC1.MatSkew(m_zhat);
	m_u = m_INTCALC1 * m_N;
	m_u.MatScale(1/m_u.VectorNorm());
	theta = asin((double)m_N.m_Value[2][0]) - theta_d;	
	
	// Rotation controller
	m_eW = m_u;
	m_eW.MatScale(theta);
	if (m_eW.VectorNorm() < 5)
	{
		m_Wc = m_lambdaw * m_eW;
		m_Wc.MatScale(-1);
	}
	else
		m_Wc.MatZeros(3,1);	
	
	// Translation controller
	m_p1.Mat2Vector(1,m_pi);
	m_m1 = m_InvCiHCalibrationMatrix * m_p1;
	m_pd1.Mat2Vector(3,m_CiHCalibrationMatrix);
	m_md1 = m_InvCiHCalibrationMatrix * m_pd1;
	m_eV = m_m1 - m_md1;
	m_eV.m_Value[2][0] = m_alpha.m_Value[0][0] - alpha_d;

	// Image Lv Matrix
	m_Lv.m_Value[0][0] = -1;
	m_Lv.m_Value[0][1] = 0;
	m_Lv.m_Value[0][2] = m_m1.m_Value[0][0];
	m_Lv.m_Value[1][0] = 0;
	m_Lv.m_Value[1][1] = -1;
	m_Lv.m_Value[1][2] = m_m1.m_Value[1][0];
	m_Lv.m_Value[2][0] = 0;
	m_Lv.m_Value[2][1] = 0;
	m_Lv.m_Value[2][2] = -1 * m_alpha.m_Value[0][0];
	m_LvInv = m_Lv^-1;

	// Image Lvww Matrix
	m_Lvw.m_Value[0][0] = m_m1.m_Value[0][0] * m_m1.m_Value[1][0];
	m_Lvw.m_Value[0][1] = -1 - (m_m1.m_Value[0][0] * m_m1.m_Value[0][0]);
	m_Lvw.m_Value[0][2] = m_m1.m_Value[1][0];
	m_Lvw.m_Value[1][0] = 1 + (m_m1.m_Value[1][0] * m_m1.m_Value[1][0]);
	m_Lvw.m_Value[1][1] = -1 * m_m1.m_Value[0][0] * m_m1.m_Value[1][0];
	m_Lvw.m_Value[1][2] = -1 * m_m1.m_Value[0][0];
	m_Lvw.m_Value[2][0] = -1 * m_alpha.m_Value[0][0] * m_m1.m_Value[1][0];
	m_Lvw.m_Value[2][1] = m_m1.m_Value[0][0];
	m_Lvw.m_Value[2][2] = 0;

	// Adaptive depth update law
	m_TranseV.MatTranspose(m_eV);
	m_INTCALC2.MatMult(m_TranseV,m_Lvw,m_Wc);	
	zstar_hat_dot = gamma * m_INTCALC2.m_Value[0][0];
	zstar_hat = zstar_hat + zstar_hat_dot*(time_ms - time_ms_OLD)/1000;
		
	m_INTCALC3 = m_lambdav * m_eV;
	m_INTCALC4 = m_Lvw * m_Wc;	
	m_INTCALC4.MatScale(zstar_hat);
	m_INTCALC5 = m_INTCALC3 + m_INTCALC4;
	m_LvInv = m_Lv^-1;
	m_Vc = m_LvInv * m_INTCALC5;	
	m_Vc.MatScale(-m_alpha.m_Value[0][0]);	
	
	// Control signals (Robot Linear and Angular Velocity Commands)
	m_INTCALC6.MatAugment("row",m_Vc,m_Wc);
	m_Vr = m_INTCALC11 * m_INTCALC6;
	m_Wr = m_INTCALC12 * m_INTCALC6;

	double eW_Norm = m_eW.VectorNorm();
	double eV_Norm = m_eV.VectorNorm();
	m_Vr.m_Value[0][0] = m_Vr.m_Value[0][0];
	m_Vr.m_Value[1][0] = m_Vr.m_Value[1][0];
	m_Vr.m_Value[2][0] = -m_Vr.m_Value[2][0];

	m_Wr.m_Value[0][0] = m_Wr.m_Value[0][0];
	m_Wr.m_Value[1][0] = m_Wr.m_Value[1][0];

	if (frame_count % 2 == 0)
	{
		printf("eW Norm: %f\n",eW_Norm);
		printf("eV Norm: %f\n",eV_Norm);
		printf("Theta: %f\n",(asin(m_N.m_Value[2][0])*180/3.1415) - theta_d);
	}
		
	Robot.ServoWithVelocity(m_Wr,m_Vr);
	time_ms_OLD = time_ms;

	m_eV.FileWrite(feV,"a","");
	m_Vr.FileWrite(fVr,"a","");
	m_eW.FileWrite(feW,"a","");
	m_Wr.FileWrite(fWr,"a","");	
	m_pi.FileWrite(fpi,"a","");
	m_N.FileWrite(fN,"a","");
	m_alpha.FileWrite(falpha,"a","");
	fprintf(fZhatStar,"%f\n",zstar_hat);
}


// **********************************************************************************
// TBZ Controller Initialization
// **********************************************************************************

void TBZController::TBZControllerInit()
{
	// Camera In-Hand (CiH) Calibration Matrix 'A'
	// Intrisic Camera Calibration Matrix	
	m_CiHCalibrationMatrix.m_Value[0][0] = 868.8675;
	m_CiHCalibrationMatrix.m_Value[0][1] = 0;
	m_CiHCalibrationMatrix.m_Value[0][2] = 345.0077;

	m_CiHCalibrationMatrix.m_Value[1][0] = 0;
	m_CiHCalibrationMatrix.m_Value[1][1] = 878.8879;
	m_CiHCalibrationMatrix.m_Value[1][2] = 245.3824;

	m_CiHCalibrationMatrix.m_Value[2][0] = 0;
	m_CiHCalibrationMatrix.m_Value[2][1] = 0;
	m_CiHCalibrationMatrix.m_Value[2][2] = 1;

	m_InvCiHCalibrationMatrix = m_CiHCalibrationMatrix^-1;

	// Extrinsic Camera Calibration Matrix (Rotation)
	m_RrExtrinsicCalibrationMatrix.m_Value[0][0] =  1;
	m_RrExtrinsicCalibrationMatrix.m_Value[0][1] =  0;
	m_RrExtrinsicCalibrationMatrix.m_Value[0][2] =  0;
	m_RrExtrinsicCalibrationMatrix.m_Value[1][0] =  0;
	m_RrExtrinsicCalibrationMatrix.m_Value[1][1] =  0.9974;
	m_RrExtrinsicCalibrationMatrix.m_Value[1][2] =  0.0724;
	m_RrExtrinsicCalibrationMatrix.m_Value[2][0] =  0;
	m_RrExtrinsicCalibrationMatrix.m_Value[2][1] = -0.0724;
	m_RrExtrinsicCalibrationMatrix.m_Value[2][2] =  0.9974;

	// Extrinsic Camera Calibration Matrix (Translation)
	// in mm
	m_trExtrinsicCalibrationMatrix.m_Value[0][0] = -100.3/25.4;
	m_trExtrinsicCalibrationMatrix.m_Value[1][0] = 48.0/25.4;
	m_trExtrinsicCalibrationMatrix.m_Value[2][0] = -48.0/25.4;

	// Calculate the Extrinsic Calibration Matrix
	m_trSkewExtrinsicCalibrationMatrix.MatSkew(m_trExtrinsicCalibrationMatrix);
	m_INTCALC10 = m_trSkewExtrinsicCalibrationMatrix * m_RrExtrinsicCalibrationMatrix;
	m_Zeros.MatZeros(3,3);

	m_INTCALC11.MatAugment("col", m_RrExtrinsicCalibrationMatrix, m_INTCALC10);
	m_INTCALC12.MatAugment("col", m_Zeros, m_RrExtrinsicCalibrationMatrix);
	m_ExtrinsicCalibrationMatrix.MatAugment("row",m_INTCALC11,m_INTCALC12);

	m_InvExtrinsicCalibrationMatrix = m_ExtrinsicCalibrationMatrix^-1;
	m_INTCALC11.MatDeAugment("row",3,1,m_InvExtrinsicCalibrationMatrix);
	m_INTCALC12.MatDeAugment("row",3,2,m_InvExtrinsicCalibrationMatrix);

	// Controller gain
	m_lambdaw.MatZeros(3,3);
	m_lambdaw.m_Value[0][0] = 0.05; // 0.05 0.05 0.03
	m_lambdaw.m_Value[1][1] = 0.03;
	m_lambdaw.m_Value[2][2] = 0.02;

	m_lambdav.MatZeros(3,3);
	m_lambdav.m_Value[0][0] = 12; // 5 5 3
	m_lambdav.m_Value[1][1] = 12;
	m_lambdav.m_Value[2][2] = 5;

	gamma = 5;
	zstar_hat = 10;

	// Desired regulation point
	theta_d = 40*3.1415/180;	
	alpha_d = 1.3;		

	// Number of feature points for homography decomposition
	numPoints = 4;

}


// **********************************************************************************
// Read Target Pixel Values for Camera in-Hand (CiH) and Fixed Camera (CF)
// **********************************************************************************

void TBZController::TBZTargetCoord(CvPoint2D32f *points[2], CvPoint2D32f *DesiredPts[2], int frame_count)
{	

	if (flagFirstFrame == 1)
	{
		// Pixel coordinates taken using the stationary camera with zooming capabilities
		m_pi_star.m_Value[0][0] = DesiredPts[1][0].x;
		m_pi_star.m_Value[1][0] = DesiredPts[1][0].y;
		m_pi_star.m_Value[2][0] = 1;
		m_pi_star.m_Value[0][1] = DesiredPts[1][1].x;
		m_pi_star.m_Value[1][1] = DesiredPts[1][1].y;
		m_pi_star.m_Value[2][1] = 1;
		m_pi_star.m_Value[0][2] = DesiredPts[1][2].x;
		m_pi_star.m_Value[1][2] = DesiredPts[1][2].y;
		m_pi_star.m_Value[2][2] = 1;
		m_pi_star.m_Value[0][3] = DesiredPts[1][3].x;
		m_pi_star.m_Value[1][3] = DesiredPts[1][3].y;
		m_pi_star.m_Value[2][3] = 1;

		flagFirstFrame = 0;
	}

		// Pixel coordinates of the planar object taken by moving camera
		m_pi.m_Value[0][0] = points[1][0].x;
		m_pi.m_Value[1][0] = points[1][0].y;
		m_pi.m_Value[2][0] = 1;
		m_pi.m_Value[0][1] = points[1][1].x;
		m_pi.m_Value[1][1] = points[1][1].y;
		m_pi.m_Value[2][1] = 1;
		m_pi.m_Value[0][2] = points[1][2].x;
		m_pi.m_Value[1][2] = points[1][2].y;
		m_pi.m_Value[2][2] = 1;
		m_pi.m_Value[0][3] = points[1][3].x;
		m_pi.m_Value[1][3] = points[1][3].y;
		m_pi.m_Value[2][3] = 1;

}



// **********************************************************************************
// Create HG Workspace (Allocate Memory)
// **********************************************************************************

void TBZController::TBZHomography(int frame_count)
{
	// Allocate Memory
	TBZCreateHGWorkspace(numPoints);

	// Finding Projective Homography
	TBZGetHomographySVD();

	// Homography Matrix
	m_Hn.MatMult(m_InvCiHCalibrationMatrix, m_Gn, m_CiHCalibrationMatrix);

	// Decomposing Homography Matrix
	TBZDecomposeHomographySimple();

	// Release Memory
	TBZReleaseHGWorkspace();
}


// **********************************************************************************
// Create HG Workspace (Allocate Memory)
// **********************************************************************************

void TBZController::TBZCreateHGWorkspace(int numPoints)
{
	// For TBZGetHomographySVD Function	
	gh_work = gsl_vector_alloc(8);	
	gh_s = gsl_vector_alloc(8);
	gh_v = gsl_matrix_alloc(8,8);
	gh_x = gsl_vector_alloc(8);
	gh_u = gsl_matrix_alloc(2 * numPoints, 8);
	gh_b = gsl_vector_alloc(2 * numPoints);	

	// For TBZDecomposeHomographySimple Function
	dh_work = gsl_vector_alloc(3);
	dh_u = gsl_matrix_alloc(3,3);
	dh_s = gsl_vector_alloc(3);
	dh_v = gsl_matrix_alloc(3,3);

	m_U.CreateMemory(3,3);
	m_V.CreateMemory(3,3);
	m_n_dash.CreateMemory(3,1);
	m_R_dash.CreateMemory(3,3);
	m_D.CreateMemory(3,1);
	m_x_dash.CreateMemory(3,1);
	m_TransV.CreateMemory(3,3);
}


// **********************************************************************************
// TBZ Homography Decomposition Using SVD (Get Gn and Alpha_g33)
// **********************************************************************************

int TBZController::TBZGetHomographySVD()
{
	int numPoints = 0;
	numPoints = m_pi.Col();
	
	for (int i = 1; i <= numPoints; i++)
	{
		int row = 2 * i - 1;
		gsl_matrix_set(gh_u, row - 1, 0, m_pi_star.m_Value[0][i-1]);
		gsl_matrix_set(gh_u, row - 1, 1, m_pi_star.m_Value[1][i-1]);
		gsl_matrix_set(gh_u, row - 1, 2, 1.0);
		gsl_matrix_set(gh_u, row - 1, 3, 0.0);
		gsl_matrix_set(gh_u, row - 1, 4, 0.0);
		gsl_matrix_set(gh_u, row - 1, 5, 0.0);
		gsl_matrix_set(gh_u, row - 1, 6, -1.0 * m_pi_star.m_Value[0][i-1] * m_pi.m_Value[0][i-1]);
		gsl_matrix_set(gh_u, row - 1, 7, -1.0 * m_pi_star.m_Value[1][i-1] * m_pi.m_Value[0][i-1]);
		gsl_vector_set(gh_b, row - 1, 1.0 * m_pi.m_Value[0][i-1]);

		row = 2 * i;
		gsl_matrix_set(gh_u, row - 1, 0, 0.0);
		gsl_matrix_set(gh_u, row - 1, 1, 0.0);
		gsl_matrix_set(gh_u, row - 1, 2, 0.0);
		gsl_matrix_set(gh_u, row - 1, 3, m_pi_star.m_Value[0][i-1]);
		gsl_matrix_set(gh_u, row - 1, 4, m_pi_star.m_Value[1][i-1]);
		gsl_matrix_set(gh_u, row - 1, 5, 1.0);
		gsl_matrix_set(gh_u, row - 1, 6, -1.0 * m_pi_star.m_Value[0][i-1] * m_pi.m_Value[1][i-1]);
		gsl_matrix_set(gh_u, row - 1, 7, -1.0 * m_pi_star.m_Value[1][i-1] * m_pi.m_Value[1][i-1]);
		gsl_vector_set(gh_b, row - 1, 1.0 * m_pi.m_Value[1][i-1]);
	} 
	
	if( gsl_linalg_SV_decomp(gh_u, gh_v, gh_s, gh_work) )
		return -1;
		
	if(gsl_linalg_SV_solve(gh_u, gh_v, gh_s, gh_b, gh_x))
		return -1;

	// Extract eigen vector for the smallest eigen value
	m_Gn.m_Value[0][0] = gsl_vector_get(gh_x, 0);
	m_Gn.m_Value[0][1] = gsl_vector_get(gh_x, 1);
	m_Gn.m_Value[0][2] = gsl_vector_get(gh_x, 2);
	m_Gn.m_Value[1][0] = gsl_vector_get(gh_x, 3);
	m_Gn.m_Value[1][1] = gsl_vector_get(gh_x, 4);
	m_Gn.m_Value[1][2] = gsl_vector_get(gh_x, 5);
	m_Gn.m_Value[2][0] = gsl_vector_get(gh_x, 6);
	m_Gn.m_Value[2][1] = gsl_vector_get(gh_x, 7);
	m_Gn.m_Value[2][2] = 1.0;


	// compute alpha.G33 for each image correspondance
	for (int i = 0; i < numPoints; i++)
		m_alpha_g33.m_Value[i][0] = 1.0/(m_Gn.m_Value[2][0] * m_pi_star.m_Value[0][i] 
									+ m_Gn.m_Value[2][1] * m_pi_star.m_Value[1][i] + 1.0);
	return 0;
}


// **********************************************************************************
// Decompose Homography (Get Rotation and Translation)
// **********************************************************************************

int TBZController::TBZDecomposeHomographySimple()
{
	double sign;
	double d_dash;
	float st, ct; // sin(theta) and cos(theta). 'float' type to 
				  // avoid numerical errors
	float x1, x3;
	double scaleFactor;	// the 'd' in decomposition alg. p 290 faugeras
	int flag_AllEigEqual = 0;

	//---------------------------------------------------------
	// Equations. See Faugeras for description of the algorithm
	// R_bar = sign.U.R_dash.V^T
	// x_f_bar = U.x_dash;
	// n_star = V.n_dash;
	// d_star = sign.d_dash;
	//---------------------------------------------------------
	
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			gsl_matrix_set(dh_u, i, j, m_Hn.m_Value[i][j]);
		}
	}

	//SVD of homography matrix
	if( gsl_linalg_SV_decomp(dh_u, dh_v, dh_s, dh_work) )
		return -1;
	
	// d - definition
	for (int i = 0; i < 3; i++)
	{
		m_D.m_Value[i][0] = gsl_vector_get(dh_s, i);
		for (int j = 0; j < 3; j++)
		{
			m_U.m_Value[i][j] = gsl_matrix_get(dh_u, i, j);
			m_V.m_Value[i][j] = gsl_matrix_get(dh_v, i, j);
		}
	}
	m_TransV.MatTranspose(m_V);

	// s = det(U) * det(V)
	sign = m_U.MatDeterminant() * m_V.MatDeterminant();

	// Based on the sign of d_dash, find R_dash and x_dash
	// NOTE: Using error bounds instead. Is 0.9999 = 1.0001?
	if( (fabs(m_D.m_Value[0][0] - m_D.m_Value[1][0]) > ERROR_BOUND) && (fabs(m_D.m_Value[1][0] - m_D.m_Value[2][0]) > ERROR_BOUND) )
	{
//		printf("Eigenvalues are DISTINCT\n");

		x1 = sqrt((m_D.m_Value[0][0]*m_D.m_Value[0][0] - m_D.m_Value[1][0]*m_D.m_Value[1][0])/(m_D.m_Value[0][0]*m_D.m_Value[0][0] - m_D.m_Value[2][0]*m_D.m_Value[2][0]));
		x3 = sqrt((m_D.m_Value[1][0]*m_D.m_Value[1][0] - m_D.m_Value[2][0]*m_D.m_Value[2][0])/(m_D.m_Value[0][0]*m_D.m_Value[0][0] - m_D.m_Value[2][0]*m_D.m_Value[2][0]));

		st = sqrt((m_D.m_Value[0][0]*m_D.m_Value[0][0] - m_D.m_Value[1][0]*m_D.m_Value[1][0])*(m_D.m_Value[1][0]*m_D.m_Value[1][0] - m_D.m_Value[2][0]*m_D.m_Value[2][0]))/(m_D.m_Value[1][0] * (m_D.m_Value[0][0] + m_D.m_Value[2][0]));
		ct = (m_D.m_Value[1][0] * m_D.m_Value[1][0] + m_D.m_Value[0][0] * m_D.m_Value[2][0])/(m_D.m_Value[1][0] * (m_D.m_Value[0][0] + m_D.m_Value[2][0]));		

		// d
		d = sign*m_D.m_Value[1][0];
		
		// --------------------------------- Solution # 1
		// np
		m_nstarActual.m_Value[0][0] = x1;
		m_nstarActual.m_Value[1][0] = 0;
		m_nstarActual.m_Value[2][0] = x3;

		// N
		m_N1 = m_V*m_nstarActual;

		// Rp
		m_R_dash.m_Value[0][0] = ct;
		m_R_dash.m_Value[0][1] = 0;
		m_R_dash.m_Value[0][2] = -1*st;
		m_R_dash.m_Value[1][0] = 0;
		m_R_dash.m_Value[1][1] = 1;
		m_R_dash.m_Value[1][2] = 0;
		m_R_dash.m_Value[2][0] = st;
		m_R_dash.m_Value[2][1] = 0;
		m_R_dash.m_Value[2][2] = ct;

		m_R1.MatMult(m_U, m_R_dash, m_TransV);
		m_R1 = m_R1 * sign;
		
		// Rn
		m_Rn.m_Value[2][0] = m_R1.m_Value[2][0]*m_N1.m_Value[0][0] + m_R1.m_Value[2][1]*m_N1.m_Value[1][0] + m_R1.m_Value[2][2]*m_N1.m_Value[2][0];
		m_Rf.m_Value[2][0] = m_R1.m_Value[0][2]*m_N1.m_Value[0][0] + m_R1.m_Value[1][2]*m_N1.m_Value[1][0] + m_R1.m_Value[2][2]*m_N1.m_Value[2][0];

		// tp
		m_x_dash.m_Value[0][0] = (m_D.m_Value[0][0] - m_D.m_Value[2][0]) * x1;
		m_x_dash.m_Value[1][0] = 0;
		m_x_dash.m_Value[2][0] = -1 * (m_D.m_Value[0][0] - m_D.m_Value[2][0]) * x3;

		// T
		m_T1 = m_U * m_x_dash;

		// --------------------------------- Solution # 2
		// np
		m_nstarActual.m_Value[0][0] = -1*x1;
		m_nstarActual.m_Value[1][0] = 0;
		m_nstarActual.m_Value[2][0] = x3;

		// N
		m_N2 = m_V*m_nstarActual;

		// Rp
		m_R_dash.m_Value[0][0] = ct;
		m_R_dash.m_Value[0][1] = 0;
		m_R_dash.m_Value[0][2] = st;
		m_R_dash.m_Value[1][0] = 0;
		m_R_dash.m_Value[1][1] = 1;
		m_R_dash.m_Value[1][2] = 0;
		m_R_dash.m_Value[2][0] = -1*st;
		m_R_dash.m_Value[2][1] = 0;
		m_R_dash.m_Value[2][2] = ct;

		m_R2.MatMult(m_U, m_R_dash, m_TransV);
		m_R2 = m_R2 * sign;
		
		// Rn
		m_Rn.m_Value[2][1] = m_R2.m_Value[2][0]*m_N2.m_Value[0][0] + m_R2.m_Value[2][1]*m_N2.m_Value[1][0] + m_R2.m_Value[2][2]*m_N2.m_Value[2][0];
		m_Rf.m_Value[2][1] = m_R2.m_Value[0][2]*m_N2.m_Value[0][0] + m_R2.m_Value[1][2]*m_N2.m_Value[1][0] + m_R2.m_Value[2][2]*m_N2.m_Value[2][0];

		// tp
		m_x_dash.m_Value[0][0] = -1 * (m_D.m_Value[0][0] - m_D.m_Value[2][0]) * x1;
		m_x_dash.m_Value[1][0] = 0;
		m_x_dash.m_Value[2][0] = -1 * (m_D.m_Value[0][0] - m_D.m_Value[2][0]) * x3;

		// T
		m_T2 = m_U * m_x_dash;

		// --------------------------------- Solution # 3
		// np
		m_nstarActual.m_Value[0][0] = x1;
		m_nstarActual.m_Value[1][0] = 0;
		m_nstarActual.m_Value[2][0] = -1*x3;

		// N
		m_N3 = m_V*m_nstarActual;

		// Rp
		m_R_dash.m_Value[0][0] = ct;
		m_R_dash.m_Value[0][1] = 0;
		m_R_dash.m_Value[0][2] = st;
		m_R_dash.m_Value[1][0] = 0;
		m_R_dash.m_Value[1][1] = 1;
		m_R_dash.m_Value[1][2] = 0;
		m_R_dash.m_Value[2][0] = -1*st;
		m_R_dash.m_Value[2][1] = 0;
		m_R_dash.m_Value[2][2] = ct;

		m_R3.MatMult(m_U, m_R_dash, m_TransV);
		m_R3 = m_R3 * sign;
		
		// Rn
		m_Rn.m_Value[2][2] = m_R3.m_Value[2][0]*m_N3.m_Value[0][0] + m_R3.m_Value[2][1]*m_N3.m_Value[1][0] + m_R3.m_Value[2][2]*m_N3.m_Value[2][0];
		m_Rf.m_Value[2][2] = m_R3.m_Value[0][2]*m_N3.m_Value[0][0] + m_R3.m_Value[1][2]*m_N3.m_Value[1][0] + m_R3.m_Value[2][2]*m_N3.m_Value[2][0];

		// tp
		m_x_dash.m_Value[0][0] = (m_D.m_Value[0][0] - m_D.m_Value[2][0]) * x1;
		m_x_dash.m_Value[1][0] = 0;
		m_x_dash.m_Value[2][0] = (m_D.m_Value[0][0] - m_D.m_Value[2][0]) * x3;

		// T
		m_T3 = m_U * m_x_dash;

		// --------------------------------- Solution # 4
		// np
		m_nstarActual.m_Value[0][0] = -1*x1;
		m_nstarActual.m_Value[1][0] = 0;
		m_nstarActual.m_Value[2][0] = -1*x3;

		// N
		m_N4 = m_V*m_nstarActual;

		// Rp
		m_R_dash.m_Value[0][0] = ct;
		m_R_dash.m_Value[0][1] = 0;
		m_R_dash.m_Value[0][2] = -1*st;
		m_R_dash.m_Value[1][0] = 0;
		m_R_dash.m_Value[1][1] = 1;
		m_R_dash.m_Value[1][2] = 0;
		m_R_dash.m_Value[2][0] = st;
		m_R_dash.m_Value[2][1] = 0;
		m_R_dash.m_Value[2][2] = ct;

		m_R4.MatMult(m_U, m_R_dash, m_TransV);
		m_R4 = m_R4 * sign;
		
		// Rn
		m_Rn.m_Value[2][3] = m_R4.m_Value[2][0]*m_N4.m_Value[0][0] + m_R4.m_Value[2][1]*m_N4.m_Value[1][0] + m_R4.m_Value[2][2]*m_N4.m_Value[2][0];
		m_Rf.m_Value[2][3] = m_R4.m_Value[0][2]*m_N4.m_Value[0][0] + m_R4.m_Value[1][2]*m_N4.m_Value[1][0] + m_R4.m_Value[2][2]*m_N4.m_Value[2][0];

		// tp
		m_x_dash.m_Value[0][0] = -1 * (m_D.m_Value[0][0] - m_D.m_Value[2][0]) * x1;
		m_x_dash.m_Value[1][0] = 0;
		m_x_dash.m_Value[2][0] = (m_D.m_Value[0][0] - m_D.m_Value[2][0]) * x3;

		// T
		m_T4 = m_U * m_x_dash;

		// Reduce solutions - features in front of the camera
		int i,j = 0;
		for(i=0; i<4; i++)
		{
			if (m_Rn.m_Value[2][i] > 0)
			{	
				m_SolutionSet.m_Value[j][0] = i;
				j = j + 1;
			}				
		}

		// Chose solution by nz
		if (j == 1)	
		{
			i = m_SolutionSet.m_Value[j][0];
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
			int i1 = m_SolutionSet.m_Value[0][0];
			int i2 = m_SolutionSet.m_Value[1][0];
			if (m_Rn.m_Value[2][i1] > m_Rn.m_Value[2][i2])
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
	if( (fabs(m_D.m_Value[0][0] - m_D.m_Value[1][0]) <= ERROR_BOUND) && (fabs(m_D.m_Value[1][0] - m_D.m_Value[2][0]) <= ERROR_BOUND) )
	{		
//		printf("Eigenvalues are EQUAL\n");

		// d
		d = 0;

		// R
		m_R = m_Hn;

		// T
		m_T.m_Value[0][0] = 0;
		m_T.m_Value[1][0] = 0;
		m_T.m_Value[2][0] = 0;

		// N
		m_N.m_Value[0][0] = 0;
		m_N.m_Value[1][0] = 0;
		m_N.m_Value[2][0] = 0;

		flag_AllEigEqual = 1;
	}

	// two equal singular values (translation is normal to the plane)
	if (flag_AllEigEqual == 0)
	{
		if( (fabs(m_D.m_Value[0][0] - m_D.m_Value[1][0]) <= ERROR_BOUND) || (fabs(m_D.m_Value[1][0] - m_D.m_Value[2][0]) <= ERROR_BOUND))
		{
//			printf("TWO eigenvalues are EQUAL\n");

			// d
			d = sign*m_D.m_Value[1][0];
			
			// --------------------------------- Solution # 1
			// np
			m_nstarActual.m_Value[0][0] = 0;
			m_nstarActual.m_Value[1][0] = 0;
			m_nstarActual.m_Value[2][0] = 1;

			// N
			m_N1 = m_V*m_nstarActual;

			// Rp
			m_R_dash.m_Value[0][0] = 1;
			m_R_dash.m_Value[0][1] = 0;
			m_R_dash.m_Value[0][2] = 0;
			m_R_dash.m_Value[1][0] = 0;
			m_R_dash.m_Value[1][1] = 1;
			m_R_dash.m_Value[1][2] = 0;
			m_R_dash.m_Value[2][0] = 0;
			m_R_dash.m_Value[2][1] = 0;
			m_R_dash.m_Value[2][2] = 1;

			m_R1.MatMult(m_U, m_R_dash, m_TransV);
			m_R1 = m_R1 * sign;
			
			// Rn & Rf
			m_Rn.m_Value[2][0] = m_R1.m_Value[2][0]*m_N1.m_Value[0][0] + m_R1.m_Value[2][1]*m_N1.m_Value[1][0] + m_R1.m_Value[2][2]*m_N1.m_Value[2][0];
			m_Rf.m_Value[2][0] = m_R1.m_Value[0][2]*m_N1.m_Value[0][0] + m_R1.m_Value[1][2]*m_N1.m_Value[1][0] + m_R1.m_Value[2][2]*m_N1.m_Value[2][0];

			// tp
			m_x_dash.m_Value[0][0] = 0;
			m_x_dash.m_Value[1][0] = 0;
			m_x_dash.m_Value[2][0] = (m_D.m_Value[2][0] - m_D.m_Value[0][0]);

			// T
			m_T1 = m_U * m_x_dash;

			// --------------------------------- Solution # 2
			// np
			m_nstarActual.m_Value[0][0] = 0;
			m_nstarActual.m_Value[1][0] = 0;
			m_nstarActual.m_Value[2][0] = -1;

			// N
			m_N2 = m_V * m_nstarActual;

			// Rp
			m_R_dash.m_Value[0][0] = 1;
			m_R_dash.m_Value[0][1] = 0;
			m_R_dash.m_Value[0][2] = 0;
			m_R_dash.m_Value[1][0] = 0;
			m_R_dash.m_Value[1][1] = 1;
			m_R_dash.m_Value[1][2] = 0;
			m_R_dash.m_Value[2][0] = 0;
			m_R_dash.m_Value[2][1] = 0;
			m_R_dash.m_Value[2][2] = 1;

			m_R2.MatMult(m_U, m_R_dash, m_TransV);
			m_R2 = m_R2 * sign;
			
			// Rn & Rf
			m_Rn.m_Value[2][1] = m_R2.m_Value[2][0]*m_N2.m_Value[0][0] + m_R2.m_Value[2][1]*m_N2.m_Value[1][0] + m_R2.m_Value[2][2]*m_N2.m_Value[2][0];
			m_Rf.m_Value[2][1] = m_R2.m_Value[0][2]*m_N2.m_Value[0][0] + m_R2.m_Value[1][2]*m_N2.m_Value[1][0] + m_R2.m_Value[2][2]*m_N2.m_Value[2][0];

			// tp
			m_x_dash.m_Value[0][0] = 0;
			m_x_dash.m_Value[1][0] = 0;
			m_x_dash.m_Value[2][0] = -1 * (m_D.m_Value[2][0] - m_D.m_Value[0][0]);

			// T
			m_T2 = m_U * m_x_dash;
			
			// Reduce solutions - features in front of the camera
			int i,j = 0;
			for(i=0; i<2; i++)
			{
				if (m_Rn.m_Value[2][i] > 0)
				{	
					m_SolutionSet.m_Value[j][0] = i;
					j = j + 1;
				}				
			}

			// Chose solution by nz
			if (j == 1)	
			{
				i = (int)m_SolutionSet.m_Value[j][0];
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
				int i1 = m_SolutionSet.m_Value[0][0];
				int i2 = m_SolutionSet.m_Value[1][0];
				if (m_Rf.m_Value[2][i1] > m_Rf.m_Value[2][i2])
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
	
	for(int index = 0; index < m_alpha_g33.GetNumElements(); index++)
		m_alpha.m_Value[index][0] = m_alpha_g33.m_Value[index][0] * d;
	
	return 0;
}


// **********************************************************************************
// De-Allocate Memory of HG Workspace
// **********************************************************************************

void TBZController::TBZReleaseHGWorkspace()
{
	// For TBZGetHomographySVD Function
	gsl_vector_free(gh_x);
	gsl_matrix_free(gh_v);
	gsl_vector_free(gh_s);
	gsl_vector_free(gh_work);
	gsl_matrix_free(gh_u);
	gsl_vector_free(gh_b);

	// For TBZDecomposeHomographySimple Function
	gsl_matrix_free(dh_u);
	gsl_matrix_free(dh_v);
	gsl_vector_free(dh_s);
	gsl_vector_free(dh_work);
	
	m_U.DeleteMemory();
	m_V.DeleteMemory();
	m_n_dash.DeleteMemory();
	m_R_dash.DeleteMemory();
	m_D.DeleteMemory();
	m_x_dash.DeleteMemory();
	m_TransV.DeleteMemory();
}


// **********************************************************************************
// Create Fixed TBZ Workspace (Allocate Memory)
// **********************************************************************************

void TBZController::TBZCreateFixedTBZWorkspace()
{	
	fZhatStar = fopen("zHatStar.txt","a");
	fTime = fopen("timeData.txt","a");
	feV = fopen("TranslationError.txt","a");	
	fVr = fopen("LinearVel.txt","a");
	feW = fopen("RotationError.txt","a");
	fWr = fopen("RotationVel.txt","a");
	fpi = fopen("PixelCoordinates.txt","a");
	fN = fopen("UnitNormal.txt","a");
	falpha = fopen("Alpha.txt","a");

	// For TBZTargetCoord Function
	m_pi_star.CreateMemory (3,4);
	m_pi.CreateMemory (3,4);

	m_CiHCalibrationMatrix.CreateMemory (3,3);
	m_InvCiHCalibrationMatrix.CreateMemory(3,3);
	m_ExtrinsicCalibrationMatrix.CreateMemory(6,6);
	m_InvExtrinsicCalibrationMatrix.CreateMemory(6,6);

	m_nstarActual.CreateMemory(3,1);

	m_lambdaw.CreateMemory(3,3);
	m_lambdav.CreateMemory(3,3);

	m_Wc.CreateMemory(3,1);
	m_Vc.CreateMemory(3,1);
	m_Wr.CreateMemory(3,1);
	m_Vr.CreateMemory(3,1);

	m_RrExtrinsicCalibrationMatrix.CreateMemory(3,3);
	m_trExtrinsicCalibrationMatrix.CreateMemory(3,3);
	m_trSkewExtrinsicCalibrationMatrix.CreateMemory(3,3);
	m_INTCALC10.CreateMemory(3,3);
	m_INTCALC11.CreateMemory(3,6);
	m_INTCALC12.CreateMemory(3,6);
	m_Zeros.CreateMemory(3,3);

		// For TBZHomography Function
	m_Gn.CreateMemory(3,3);
	m_Hn.CreateMemory(3,3);
	m_dG.CreateMemory(3,3);
	m_alpha_g33.CreateMemory(4,1);
	m_alpha.CreateMemory(4,1);
	m_T1.CreateMemory(3,1);
	m_T2.CreateMemory(3,1);
	m_T3.CreateMemory(3,1);
	m_T4.CreateMemory(3,1);
	m_xf_star.CreateMemory(3,1);
	m_R_star.CreateMemory(3,3);
	m_R1.CreateMemory(3,3);
	m_R2.CreateMemory(3,3);
	m_R3.CreateMemory(3,3);
	m_R4.CreateMemory(3,3);
	m_N1.CreateMemory(3,1);
	m_N2.CreateMemory(3,1);
	m_N3.CreateMemory(3,1);
	m_N4.CreateMemory(3,1);
	m_Rn.CreateMemory(3,4);
	m_Rf.CreateMemory(3,4);
	m_SolutionSet.CreateMemory(4,1);
	m_R.CreateMemory(3,3);
	m_T.CreateMemory(3,1);
	m_N.CreateMemory(3,1);

	// For TBZMainControl Function	
	m_eW.CreateMemory(3,1);
	m_eV.CreateMemory(3,1);
	m_u.CreateMemory(3,3);
	m_Lv.CreateMemory(3,3);
	m_Lvw.CreateMemory(3,3);
	m_INTCALC1.CreateMemory(3,3);
	m_INTCALC2.CreateMemory(1,1);
	m_INTCALC3.CreateMemory(3,1);
	m_INTCALC4.CreateMemory(3,1);
	m_INTCALC5.CreateMemory(3,1);
	m_INTCALC6.CreateMemory(6,1);								
	m_zhat.CreateMemory(3,1);
	m_zdhat.CreateMemory(3,1);	
	m_p1.CreateMemory(3,1);
	m_m1.CreateMemory(3,1);
	m_pd1.CreateMemory(3,1);
	m_md1.CreateMemory(3,1);
	m_LvInv.CreateMemory(3,3);
	m_TranseV.CreateMemory(1,3);
}

// **********************************************************************************
// Release Fixed TBZ Workspace (De-Allocate Memory)
// **********************************************************************************

void TBZController::TBZReleaseFixedTBZWorkspace()
{
	fclose(fTime);
	fclose(fZhatStar);
	fclose(feV);
	fclose(fVr);
	fclose(feW);
	fclose(fWr);
	fclose(fpi);
	fclose(fN);
	fclose(falpha);

	m_pi_star.DeleteMemory();
	m_pi.DeleteMemory();

	m_CiHCalibrationMatrix.DeleteMemory();
	m_InvCiHCalibrationMatrix.DeleteMemory();
	m_ExtrinsicCalibrationMatrix.DeleteMemory();
	m_InvExtrinsicCalibrationMatrix.DeleteMemory();

	m_nstarActual.DeleteMemory();

	m_Wc.DeleteMemory();
	m_Vc.DeleteMemory();
	m_Wr.DeleteMemory();
	m_Vr.DeleteMemory();

	m_RrExtrinsicCalibrationMatrix.DeleteMemory();
	m_trExtrinsicCalibrationMatrix.DeleteMemory();
	m_trSkewExtrinsicCalibrationMatrix.DeleteMemory();
	m_Zeros.DeleteMemory();	

		m_Gn.DeleteMemory();
	m_Hn.DeleteMemory();
	m_dG.DeleteMemory();
	m_alpha_g33.DeleteMemory();
	m_alpha.DeleteMemory();
	m_T1.DeleteMemory();
	m_T2.DeleteMemory();
	m_T3.DeleteMemory();
	m_T4.DeleteMemory();
	m_xf_star.DeleteMemory();
	m_R_star.DeleteMemory();
	m_R1.DeleteMemory();
	m_R2.DeleteMemory();
	m_R3.DeleteMemory();
	m_R4.DeleteMemory();
	m_N1.DeleteMemory();
	m_N2.DeleteMemory();
	m_N3.DeleteMemory();
	m_N4.DeleteMemory();
	m_Rn.DeleteMemory();
	m_Rf.DeleteMemory();
	m_SolutionSet.DeleteMemory();
	m_R.DeleteMemory();
	m_T.DeleteMemory();
	m_N.DeleteMemory();	
			
	m_eW.DeleteMemory();
	m_eV.DeleteMemory();
	m_u.DeleteMemory();
	m_Lv.DeleteMemory();
	m_Lvw.DeleteMemory();
	m_INTCALC1.DeleteMemory();
	m_INTCALC2.DeleteMemory();
	m_INTCALC3.DeleteMemory();
	m_INTCALC4.DeleteMemory();
	m_INTCALC5.DeleteMemory();
	m_INTCALC6.DeleteMemory();
	m_zhat.DeleteMemory();
	m_zdhat.DeleteMemory();	
	m_p1.DeleteMemory();
	m_m1.DeleteMemory();
	m_pd1.DeleteMemory();
	m_md1.DeleteMemory();
	m_LvInv.DeleteMemory();
	m_TranseV.DeleteMemory();

	m_R_Trans.DeleteMemory();				
	m_p1_star.DeleteMemory();
	// fclose(fTime);
}


// **********************************************************************************
// Write Results in Text File
// **********************************************************************************

void TBZController::TBZResultFileWrite(int frame_count)
{
//	FILE *fp;
//	fp = fopen("Results.txt","a");
//	fprintf(fp,"Frame # %d\n\n",frame_count);
//	for(int r=0; r<3; r++ )
//	{
//		for(int c=0; c<3; c++ )
//		{
//			fprintf(fp,"%f\t",m_Vc.m_Value[r][c]);
//		}
//		fprintf(fp,"\n");
//	}
//	fprintf(fp,"\n");
//	fclose(fp);

/*	m_R.FileWrite("Results.txt","a","Rotation Matrix");
	m_x_h.FileWrite("Results.txt","a","Translation Vector");
	m_alpha.FileWrite("Results.txt","a","Depth Ratio");
	m_eW.FileWrite("Results.txt","a","Rotation Error");
	m_eV.FileWrite("Results.txt","a","Translation Error"); */
//	m_Vc.FileWrite("Results.txt","a","Camera Linear Velocity"); 
//	m_Wc.FileWrite("Results.txt","a","Camera Angular Velocity");
//	m_Vr.FileWrite("Results.txt","a","Robot Linear Velocity");
//	m_Wr.FileWrite("Results.txt","a","Robot Angular Velocity");
//	m_pi.FileWrite("Results.txt","a","pi");
//	m_pi_star.FileWrite("Results.txt","a","pi_star");
}

// **********************************************************************************
// Program End
// **********************************************************************************
