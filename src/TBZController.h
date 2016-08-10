// **********************************************************************************
// Program Written by Siddhartha Satish Mehta
// Email: siddhart@ufl.edu
// Teach by Zooming Controller for Visual Serco Control
// Last Modified on 11/17/2005
// **********************************************************************************

#pragma once

#include "cv.h"
#include "highgui.h"
#include "LinearAlgebra.h"

class TBZController
{
public: 
// Functions
void TBZControlMain(CvPoint2D32f* points[2], CvPoint2D32f *DesiredPts[2], int frame_count);
void TBZControllerInit();
void TBZTargetCoord(CvPoint2D32f* points[2], CvPoint2D32f *DesiredPts[2], int frame_count);
void TBZCreateTBZWorkspace();
void TBZReleaseTBZWorkspace();
void TBZCreateFixedTBZWorkspace();
void TBZReleaseFixedTBZWorkspace();

void TBZHomography(int frame_count);
void TBZCreateHGWorkspace(int);
void TBZReleaseHGWorkspace();
int TBZGetHomographySVD();
int TBZDecomposeHomographySimple();
void TBZResultFileWrite(int frame_count);

// TBZControllerInit Objects
LinearAlgebra m_CiHCalibrationMatrix;
LinearAlgebra m_InvCiHCalibrationMatrix;
LinearAlgebra m_CFCalibrationMatrix;
LinearAlgebra m_InvCFCalibrationMatrix;
LinearAlgebra m_ExtrinsicCalibrationMatrix;
LinearAlgebra m_InvExtrinsicCalibrationMatrix;
LinearAlgebra m_INTCALC10;
LinearAlgebra m_INTCALC11;
LinearAlgebra m_INTCALC12;

// TBZControlMain Objects (Control Signals)
LinearAlgebra m_Wc;
LinearAlgebra m_Vc;
LinearAlgebra m_Wr;
LinearAlgebra m_Vr;
LinearAlgebra m_lambdaw;
LinearAlgebra m_lambdav;

double m_CiHDistortion[6];
double m_CFDistortion[6];
double theta_d;
FILE *fTime;
FILE *fZhatStar;
FILE* feV;
FILE* fVr;
FILE* feW;
FILE* fWr;
FILE* fpi;
FILE* fN;
FILE* falpha;

private:

	// Variables
	int numPoints;
	float d;
	double gamma;		
	double theta;	
	double alpha_d;	
	double zstar_hat;
	double zstar_hat_dot;
	int time_ms_OLD;

	// TBZTargetCoord Objects 
	LinearAlgebra m_pi;
	LinearAlgebra m_pi_star;
	LinearAlgebra m_pd1;
	LinearAlgebra m_md1;

	// TBZHomography Objects
	LinearAlgebra m_Gn;
	LinearAlgebra m_Hn;
	LinearAlgebra m_dG;
	LinearAlgebra m_alpha_g33;
	LinearAlgebra m_T1;
	LinearAlgebra m_T2;
	LinearAlgebra m_T3;
	LinearAlgebra m_T4;
	LinearAlgebra m_nstarActual;
	LinearAlgebra m_xf_star;
	LinearAlgebra m_R_star;
	LinearAlgebra m_R1;
	LinearAlgebra m_R2;
	LinearAlgebra m_R3;
	LinearAlgebra m_R4;	
	LinearAlgebra m_SolutionSet;
	LinearAlgebra m_R;
	LinearAlgebra m_T;
	LinearAlgebra m_N;

	// TBZDecomposeHomographySimple Objects
	LinearAlgebra m_U;
	LinearAlgebra m_V;
	LinearAlgebra m_n_dash;
	LinearAlgebra m_R_dash;
	LinearAlgebra m_D;
	LinearAlgebra m_x_dash;
	LinearAlgebra m_TransV;
	LinearAlgebra m_alpha;
	LinearAlgebra m_N1;
	LinearAlgebra m_N2;
	LinearAlgebra m_N3;
	LinearAlgebra m_N4;
	LinearAlgebra m_Rn;
	LinearAlgebra m_Rf;

	// TBZControlMain Objects	
	LinearAlgebra m_u;
	LinearAlgebra m_eW;
	LinearAlgebra m_eV;
	LinearAlgebra m_zhat;
	LinearAlgebra m_zdhat;
	LinearAlgebra m_Lv;
	LinearAlgebra m_Lvw;
	LinearAlgebra m_TranseV;

	LinearAlgebra m_R_Trans;
	LinearAlgebra m_m1;
	LinearAlgebra m_m1_star;
	LinearAlgebra m_md1_star;
	LinearAlgebra m_p1;
	LinearAlgebra m_p1_star;	
	LinearAlgebra m_LvInv;
	LinearAlgebra m_INTCALC1;
	LinearAlgebra m_INTCALC2;
	LinearAlgebra m_INTCALC3;
	LinearAlgebra m_INTCALC4;
	LinearAlgebra m_INTCALC5;
	LinearAlgebra m_INTCALC6;
	LinearAlgebra m_Zeros;
	LinearAlgebra m_RrExtrinsicCalibrationMatrix;
	LinearAlgebra m_trExtrinsicCalibrationMatrix;
	LinearAlgebra m_trSkewExtrinsicCalibrationMatrix;
	
};