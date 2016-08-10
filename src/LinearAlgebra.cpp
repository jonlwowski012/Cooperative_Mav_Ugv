#pragma once

//#include "stdafx.h"
#include <stdio.h>
#include <iostream>
#include "LinearAlgebra.h"
#include <math.h>
#define NMAX 100


//============================================================
// Constructor
//============================================================
LinearAlgebra::LinearAlgebra(void)
{
}


//============================================================
// Destructor
//============================================================
LinearAlgebra::~LinearAlgebra(void)
{		
}



//============================================================
// Creates memory for matrix
// NOTE: a vector is just a special case of a matrix
//============================================================
void LinearAlgebra::CreateMemory(int numOfRows, int numOfCol)
{
	int i;

    m_ROW = numOfRows;	//m
	m_COL = numOfCol;	//n


	//--------------------------------------------
	//mxn
	m_Value = new double*[numOfRows];	
	m_TempVal = new double*[numOfRows];
	m_TempVal2 = new double*[numOfRows];
	m_U = new double*[numOfRows];
	
	//nxm
	m_TransposeVal = new double*[numOfCol];
	m_InverseVal = new double*[numOfCol];	

	//nxn
	m_TempValnxn = new double*[numOfCol];
	m_V = new double*[numOfCol];

	// standard vectors
	m_W = new double[numOfCol];	
	m_rv1 = new double[numOfCol];
	

	//--------------------------------------------
	//mxn
	for(i=0; i<numOfRows; i++)
	{
		m_Value[i] = new double[numOfCol];
		m_TempVal[i] = new double[numOfCol];	
		m_TempVal2[i] = new double[numOfCol];		
		m_U[i] = new double[numOfCol];		
	}

	//nxm
	for(i=0; i<numOfCol; i++)
	{
		m_TransposeVal[i] = new double[numOfRows];
		m_InverseVal[i] = new double[numOfRows];
	}

	//nxn
	for(i=0; i<numOfCol; i++)
	{
		m_TempValnxn[i] = new double[numOfCol]; 
		m_V[i] = new double[numOfCol];
	}
}


//============================================================
// Deletes memory for matrix
//============================================================
void LinearAlgebra::DeleteMemory(void)
{
	int i;

	//mxn
	for(i=0; i<m_ROW; i++)
	{
		delete m_Value[i];
		delete m_TempVal[i];	
		delete m_TempVal2[i];		
		delete m_U[i];		
	}

	//nxm
	for(i=0; i<m_COL; i++)
	{
		delete m_TransposeVal[i];
		delete m_InverseVal[i];
	}

	//nxn
	for(i=0; i<m_COL; i++)
	{
		delete m_TempValnxn[i]; 
		delete m_V[i];
	}

	//mxn
	delete m_Value;	
	delete m_TempVal;
	delete m_TempVal2;
	delete m_U;

	//nxm
	delete m_TransposeVal;
	delete m_InverseVal;	

	//nxn
	delete m_TempValnxn;
	delete m_V;

	// standard vectors
	delete m_W;	
	delete m_rv1;
}


//============================================================
// Returns the number of rows
//============================================================
int LinearAlgebra::Row(void)
{
	return m_ROW;
}


//============================================================
// Returns the number of columns
//============================================================
int LinearAlgebra::Col(void)
{
	return m_COL;
}


//============================================================
// Returns the number of columns
//============================================================
int LinearAlgebra::GetNumElements(void)
{
	return m_ROW*m_COL;
}


//============================================================
// Overload = operator so that to matrices can be made equal
// to each other
//============================================================
void LinearAlgebra::operator = (LinearAlgebra& B)
{		
	int r, c;

	for( r=0; r<m_ROW; r++ )
		for( c=0; c<m_COL; c++ )
			m_Value[r][c] = B.m_Value[r][c];
}


//============================================================
// Overload = operator so that the values from a matrix
// math operation can be assigned to another matrix
//============================================================
void LinearAlgebra::operator = (double** PtrValue)
{		
	int r, c;

	for( r=0; r<m_ROW; r++ )
		for( c=0; c<m_COL; c++ )
			m_Value[r][c] = PtrValue[r][c];
}


//============================================================
// Matrix Addition when values are passed by an Object
//============================================================
double** LinearAlgebra::operator + (LinearAlgebra& B)
{		
	int r, c;

	for( r=0; r<m_ROW; r++ )
		for( c=0; c<m_COL; c++ )
			m_TempVal[r][c] = m_Value[r][c] + B.m_Value[r][c];
		
	return m_TempVal;
}


//============================================================
// Matrix Addition when values are passed by a double ptr to a Matrix
//============================================================
double** LinearAlgebra::operator + (double** B)
{		
	int r, c;

	for( r=0; r<m_ROW; r++ )
		for( c=0; c<m_COL; c++ )
			m_TempVal[r][c] = m_Value[r][c] + B[r][c];
		
	return m_TempVal;
}



//============================================================
// Matrix Addition by a scalar (add scalar to all elements)
//============================================================
double** LinearAlgebra::operator + (double Scalar)
{		
	int r, c;

	for( r=0; r<m_ROW; r++ )
		for( c=0; c<m_COL; c++ )
			m_TempVal[r][c] = m_Value[r][c] + Scalar;
		
	return m_TempVal;
}


//============================================================
// Matrix Subtraction when values are passed by a double ptr to a Matrix
//============================================================
double** LinearAlgebra::operator - (LinearAlgebra& B)
{		
	int r, c;

	for( r=0; r<m_ROW; r++ )
		for( c=0; c<m_COL; c++ )
			m_TempVal[r][c] = m_Value[r][c] - B.m_Value[r][c];
		
	return m_TempVal;
}



//============================================================
// Matrix Subtraction when values are passed by a double ptr to a Matrix
//============================================================
double** LinearAlgebra::operator - (double** B)
{		
	int r, c;

	for( r=0; r<m_ROW; r++ )
		for( c=0; c<m_COL; c++ )
			m_TempVal[r][c] = m_Value[r][c] - B[r][c];
		
	return m_TempVal;
}



//============================================================
// Matrix Subtraction by a scalar (subtract scalar from all elements)
//============================================================
double** LinearAlgebra::operator - (double Scalar)
{		
	int r, c;

	for( r=0; r<m_ROW; r++ )
		for( c=0; c<m_COL; c++ )
			m_TempVal[r][c] = m_Value[r][c] - Scalar;
		
	return m_TempVal;
}


//============================================================
// Negative of a matrix
//============================================================
double** LinearAlgebra::operator - (void)
{		
	int r, c;

	for( r=0; r<m_ROW; r++ )
		for( c=0; c<m_COL; c++ )
			m_TempVal[r][c] = (-1)*m_Value[r][c];
		
	return m_TempVal;
}


//============================================================
// Matrix multiplication when values are passed by a double ptr to a Matrix
//============================================================
double** LinearAlgebra::operator * (LinearAlgebra& B)
{		
	int r, c, i;
	
	for( r=0; r<m_ROW; r++ )
	{
		for( c=0; c<B.m_COL; c++ )
		{
			m_TempVal[r][c] = 0;

			for( i=0; i<m_COL; i++)
                m_TempVal[r][c] += m_Value[r][i]*B.m_Value[i][c];
		}
	}
	
	return m_TempVal;
}


//============================================================
// Three Matrix multiplication when values are passed by a double ptr to a Matrix
//============================================================
void LinearAlgebra::MatMult(LinearAlgebra& A, LinearAlgebra& B, LinearAlgebra& C)
{		
	int r, c, i;
	LinearAlgebra IP;
	IP.CreateMemory(B.m_ROW,C.m_COL);

	LinearAlgebra FP;
	FP.CreateMemory(A.m_ROW,C.m_COL);

	for( r=0; r<B.m_ROW; r++ )
	{
		for( c=0; c<C.m_COL; c++ )
		{
			IP.m_Value[r][c] = 0;
			for( i=0; i<B.m_COL; i++)
                IP.m_Value[r][c] += B.m_Value[r][i]*C.m_Value[i][c];
		}
	}
	
	for( r=0; r<A.m_ROW; r++ )
	{
		for( c=0; c<IP.m_COL; c++ )
		{
			m_Value[r][c] = 0;
			for( i=0; i<A.m_COL; i++)
                m_Value[r][c] += A.m_Value[r][i]*IP.m_Value[i][c];
		}
	}
}


//============================================================
// Matrix multiplication by a scalar
//============================================================
double** LinearAlgebra::operator * (double Scalar)
{		
	int r, c;

	
	for( r=0; r<m_ROW; r++ )
		for( c=0; c<m_COL; c++ )
			m_TempVal[r][c] = m_Value[r][c]*Scalar;
	
	return m_TempVal;
}


//============================================================
// Matrix Power and Inverse
//============================================================
double** LinearAlgebra::operator ^ (int Pwr)
{		
	int r, c, i, j;


	if( Pwr == -1 )
	{
		//matrix inverse
		Inverse();

		return m_InverseVal;
	}
	else if( Pwr > 1 )
	{	
		//do matrix power
		for( j=1; j<Pwr; j++ )
		{		
			if( j==1 )
			{
				for( r=0; r<m_ROW; r++ )
				{
					for( c=0; c<m_COL; c++ )
					{
						m_TempVal[r][c] = 0;

						for( i=0; i<m_COL; i++)
							m_TempVal[r][c] += m_Value[r][i]*m_Value[i][c];
					}
				}
			}
			else if( j%2 == 0 )
			{
				for( r=0; r<m_ROW; r++ )
				{
					for( c=0; c<m_COL; c++ )
					{
						m_TempVal2[r][c] = 0;

						for( i=0; i<m_COL; i++)
							m_TempVal2[r][c] += m_Value[r][i]*m_TempVal[i][c];
					}
				}
			}
			else
			{
				for( r=0; r<m_ROW; r++ )
				{
					for( c=0; c<m_COL; c++ )
					{
						m_TempVal[r][c] = 0;

						for( i=0; i<m_COL; i++)
							m_TempVal[r][c] += m_Value[r][i]*m_TempVal2[i][c];
					}
				}
			}
		}

		if( Pwr%2 )
			return m_TempVal2;
		else
			return m_TempVal;

	}
	else
	{
		return m_Value;
	}	
}


//============================================================
// Function for matrix transpose
// calculate the transpose
//============================================================
void LinearAlgebra::MatTranspose(LinearAlgebra& A)
{
	int r, c;

	for( r=0; r<A.m_ROW; r++ )
	{
		for( c=0; c<A.m_COL; c++ )
		{
			m_Value[c][r] = A.m_Value[r][c];
		}
	}
}



//============================================================
// Function for matrix inverse -> assume matrix is square
// calculate the inverse as V*(W^-1)*U^t
// put value in m_InverseVal
//============================================================
void LinearAlgebra::Inverse(void)
{
	int r, c, i;
	// fills in U, W, and V matrices
	SVD();

    // Temp = V*(W^-1)
	for( r=0; r<m_COL; r++ )
		for( c=0; c<m_COL; c++ )
			 m_TempValnxn[r][c] = m_V[r][c]*(1.0/m_W[c]);

	// m_InverseVal = Temp*(U^t)
	for( r=0; r<m_COL; r++ )
	{
		for( c=0; c<m_ROW; c++ )
		{
			m_InverseVal[r][c] = 0;

			for( i=0; i<m_COL; i++)
                m_InverseVal[r][c] += m_TempValnxn[r][i]*m_U[c][i];
		}
	}
}



//============================================================
// Function for matrix inverse -> assume matrix is square
// mxn matrix => m_ROWxm_COL
// A = U*W*V^t
//============================================================
void LinearAlgebra::SVD(void)
{
	int i, its, j, jj, k, p, nm;
	int flag;
	int m = m_ROW, n = m_COL;	//this is just to make algortihm easy to write
	double anorm, c, f, g, h, s, scale, x, y, z;
	double* rv1;
	double** a;	//this is the input matrix A --> it will become the matrix U when algorithm is done
	double* w;	//vector that holds the diagonal elements of the matrix W
	double** v; //this is the matrix V not V^t


	//copy Value to the U matrix so Value does not get copied over with this function
	for( i=0; i<m_ROW; i++ )
		for( j=0; j<m_COL; j++ )
			m_U[i][j] = m_Value[i][j];
	//for( i=0; i<m_ROW; i++ )
	//	memcpy(m_U[i], m_Value[i], m_COL*sizeof(double));
	
	// reassign variables -> this is just to make everything work with the "Numerical Recipes" code
	a = m_U;
	w = m_W;
	v = m_V;
	rv1 = m_rv1;


	//-----------------------------------------------
	// Householder reduction to bidiagonal form
	//-----------------------------------------------
	g = 0.0;
	scale = 0.0;
	anorm = 0.0;

	for( i=0; i<n; i++)
	{
		p = i + 1;
		rv1[i] = scale*g;
		g = 0.0;
		s = 0.0;
		scale = 0.0;

		if( i < m )
		{
			for( k=i; k<m; k++ )
				scale += fabs( a[k][i] );

			if( scale != 0.0 )
			{
				for( k=i; k<m; k++ )
				{
					a[k][i] /= scale;
					s += a[k][i]*a[k][i];
				}

				f = a[i][i];
				g = -SIGN(sqrt(s),f);	//determine sign of f, then use that to give a sign to sqrt(s)
				h = f*g - s;
				a[i][i] = f - g;

				if( i != (n-1) )
				{
					for( j=p; j<n; j++ )
					{
						s=0;
						for( k=i; k<m; k++ )
							s += a[k][i]*a[k][j];

						f = s/h;

						for( k=i; k<m; k++ )
							a[k][j] += f*a[k][i];
					}
				}

				for( k=i; k<m; k++ )
					a[k][i] *= scale;

			} //if( scale )
		} //if( i < m )

		w[i] = scale*g;
		g = 0.0;
		s = 0.0;
		scale = 0.0;

		if( (i<m) && (i!=(n-1)) )
		{
			for( k=p; k<n; k++ )
				scale += fabs( a[i][k] );

			if( scale != 0.0 )
			{
				for( k=p; k<n; k++ )
				{
					a[i][k] /= scale;
					s += a[i][k]*a[i][k];
				}

				f = a[i][p];
				g = -SIGN(sqrt(s),f);
				h = f*g - s;
				a[i][p] = f - g;

				for( k=p; k<n; k++ )
					rv1[k] = a[i][k]/h;

                if( i != (m-1) )
				{
					for( j=p; j<m; j++ )
					{
						s=0;
						for( k=p; k<n; k++ )
							s += a[j][k]*a[i][k];

						for( k=p; k<n; k++ )
							a[j][k] += s*rv1[k];
					}
				}

				for( k=p; k<n; k++ )
					a[i][k] *= scale;

			} //if( scale )
		} //if( (i<m) && (i!=(n-1)) )

		anorm = DMAX( anorm, ( fabs(w[i])+fabs(rv1[i]) ) );
	
	} //for( i=0; i<n; i++)


	//-----------------------------------------------
	// Accumulation of right-hand transformations
	//-----------------------------------------------
	for( i=(n-1); i>=0; i-- )
	{
		if( i < (n-1) )
		{
			if( g != 0.0 )
			{
				// Double division to avoid possible underflow
				for( j=p; j<n; j++ )
					v[j][i] = (a[i][j]/a[i][p])/g;
				
				for( j=p; j<n; j++ )
				{
					s=0;
					for( k=p; k<n; k++ )
						s += a[i][k]*v[k][j];

					for( k=p; k<n; k++ )
						v[k][j] += s*v[k][i];
				}
			}

			for( j=p; j<n; j++ )
			{
				v[i][j] = 0.0;
				v[j][i] = 0.0;
			}
		}

		v[i][i] = 1.0;
		g = rv1[i];
		p = i;
	}

	//-----------------------------------------------
	// Accumulation of left-hand transformations
	//-----------------------------------------------
	for( i=((int)IMIN(m,n)-1); i>=0; i-- )
	{
		p = i + 1;
		g = w[i];

		if( i < (n-1) )
		{
			for( j=p; j<n; j++ )
				a[i][j] = 0.0;
		}

		if( g != 0.0 )
		{
			g = 1.0/g;

			if( i != (n-1) )
			{
				for( j=p; j<n; j++ )
				{
					s=0;
					for( k=p; k<m; k++ )
						s += a[k][i]*a[k][j];

					f = (s/a[i][i])*g;

					for( k=i; k<m; k++ )
						a[k][j] += f*a[k][i];
				}
			}

			for( j=i; j<m; j++)
				a[j][i] *= g;
		}
		else
		{
			for( j=i; j<m; j++ )
				a[j][i] = 0.0;
		}

		//++a[i][i];
		a[i][i] += 1;
		//a[i][i]++;

	} //for( i=n-1; i>=0; i-- )


	//-----------------------------------------------
	// Diagonalization of the bidiagonal form: Loop over
	// singular values, and over allowed iterations.
	//-----------------------------------------------
	for( k=(n-1); k>=0; k--)
	{
		for( its=1; its<=30; its++ )
		{
			flag = 1;

			for( p=k; p>=0; p-- )
			{
				nm = p - 1;

				// Test for splitting -> note that rv1[0] is always zero
				if( (double)(fabs(rv1[p])+anorm) == anorm )
				{
					flag = 0;
					break;
				}

				if( (double)(fabs(w[nm])+anorm) == anorm )
					break;
			}
			if( flag )
			{
				// Cancellation of rv1[p], if p>1
				c = 0.0;
				s = 1.0;

				for( i=p; i<=k; i++ )
				{
					f = s*rv1[i];
					rv1[i] *= c;

					if( (double)(fabs(f)+anorm) == anorm )
						break;

					g = w[i];
					h = Pythag(f,g);
					w[i] = h;
					h = 1.0/h;
					c = g*h;
					s = -f*h;

					for( j=0; j<m; j++ )
					{
						y = a[j][nm];
						z = a[j][i];
						a[j][nm] = y*c + z*s;
						a[j][i] = z*c - y*s;
					}
				} //for( i=0; i<k; i++ )
			} //if( flag )

			z = w[k];

			// Convergence
			if( p == k )
			{
				// Singular value is made nonnegative
				if( z < 0.0 )
				{
					w[k] = -z;

					for( j=0; j<n; j++)
						v[j][k] *= -1;
				}

				break;
			}

			//if( its == 30 )
			//	;//error -> no convergence;

			// Shift from bottom 2-by-2 minor
			x = w[p];
			nm = k - 1;
			y = w[nm];
			g = rv1[nm];
			h = rv1[k];
			f = ( (y-z)*(y+z) + (g-h)*(g+h) )/( 2*h*y );
			g = Pythag(f,1.0);
			f = ( (x-z)*(x+z) +  h*((y/(f+SIGN(g,f))) - h) )/x;

			// Next QR transformation
			c = 1.0;
			s = 1.0;
			
			for( j=p; j<=nm; j++ )
			{
				i = j + 1;
				g = rv1[i];
				y = w[i];
				h = s*g;
				g = c*g;
				z = Pythag(f,h);
				rv1[j] = z;
				c = f/z;
				s = h/z;
				f = x*c + g*s;
				g = g*c - x*s;
				h = y*s;
				y *= c;

				for( jj=0; jj<n; jj++ )
				{
					x = v[jj][j];
					z = v[jj][i];
					v[jj][j] = x*c + z*s;
					v[jj][i] = z*c - x*s;
				}

				z = Pythag(f,h);

				// Rotation can be arbitrary if z = 0
				w[j] = z;

				if( z )
				{
					z = 1/z;
					c = f*z;
					s = h*z;
				}

				f = c*g + s*y;
				x = c*y - s*g;

				for( jj=0; jj<m; jj++ )
				{
					y = a[jj][j];
					z = a[jj][i];
					a[jj][j] = y*c + z*s;
					a[jj][i] = z*c - y*s;
				}

			} //for( j=0; j<nm; j++ )

			rv1[p] = 0;
			rv1[k] = f;
			w[k] = x;
            
		} //for( its=0; its<30; its++ )

	} //for( k=n-1; k>=0; k--)
}




//============================================================
// Computes sqrt(a^2+b^2) without destructive underflow or overflow
//============================================================
double LinearAlgebra::Pythag(double a, double b)
{
	double absa, absb;

	absa = fabs(a);
	absb = fabs(b);

	if( absa > absb )
		return absa*sqrt( 1 + SQR(absb/absa) );
	else
		return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
}



//============================================================
// Determinant of 3x3 Matrix
//============================================================
double LinearAlgebra::MatDeterminant(void)
{
	double MatDet = (m_Value[0][0]*m_Value[1][1]*m_Value[2][2]-m_Value[0][0]*m_Value[1][2]*
				   m_Value[2][1]-m_Value[1][0]*m_Value[0][1]*m_Value[2][2]+m_Value[1][0]*
				   m_Value[0][2]*m_Value[2][1]+m_Value[2][0]*m_Value[0][1]*
				   m_Value[1][2]-m_Value[2][0]*m_Value[0][2]*m_Value[1][1]);
	return MatDet;
}



//============================================================
// Displays Matrix
//============================================================

void LinearAlgebra::MatDisplay(char s[100])
{
	int r, c;
	printf("%s\n",s);

	for( r=0; r<m_ROW; r++ )
	{
		for( c=0; c<m_COL; c++ )
		{
			printf("%f\t",m_Value[r][c]);
		}
		printf("\n");
	}
	printf("\n");

}

//============================================================
// Computes Trace of the Matrix
//============================================================
double LinearAlgebra::MatTrace(void)
{
	int r, c;
	double Trace = 0.0;;

	for( r=0; r<m_ROW; r++ )
	{
		for( c=0; c<m_COL; c++ )
		{
			if (c == r)
			{
				Trace = Trace + m_Value[r][c];
			}
		}	
	}
	return Trace;
}


//============================================================
// Compute Sinc of an angle
//============================================================
double LinearAlgebra::GetSinc(double Angle)
{
	double Sinc = 0.0;
	Sinc = sin(Angle)/Angle;
	return Sinc;
}


//============================================================
// Compute Skew of a Matrix
//============================================================
void LinearAlgebra::MatSkew(LinearAlgebra &A)
{
	m_Value[0][0] = 0.0;
	m_Value[0][1] = -A.m_Value[2][0];
	m_Value[0][2] = A.m_Value[1][0];
	m_Value[1][0] = A.m_Value[2][0];
	m_Value[1][1] = 0.0;
	m_Value[1][2] = -A.m_Value[0][0];
	m_Value[2][0] = -A.m_Value[1][0];	
	m_Value[2][1] = A.m_Value[0][0];
	m_Value[2][2] = 0.0;
}


//============================================================
// Compute Anti - Skew of a Matrix (i.e. Skew Matrix to Vector)
//============================================================
void LinearAlgebra::MatAntiSkew(LinearAlgebra &A)
{
	m_Value[0][0] = A.m_Value[2][1];
	m_Value[1][0] = A.m_Value[0][2];
	m_Value[2][0] = A.m_Value[1][0];
}


//============================================================
// Matrix Scaling
//============================================================
void LinearAlgebra::MatScale(double ScaleFactor)
{
	int r,c;

	for( r=0; r<m_ROW; r++ )
	{
		for( c=0; c<m_COL; c++ )
		{
			m_Value[r][c] = ScaleFactor * m_Value[r][c];
		}	
	}
}


//============================================================
// Matrix to Vector Conversion
// A: Select column from input matrix B, which will be the column vector
//============================================================
void LinearAlgebra::Mat2Vector(int A, LinearAlgebra& B)
{
	int r;

	for( r=0; r<B.m_ROW; r++ )
	{
		m_Value[r][0] = B.m_Value[r][A-1];
	}
}


//============================================================
// Vector Norm
//============================================================
double LinearAlgebra::VectorNorm()
{
	int r;
	double Norm = 0.0;

	for( r=0; r<m_ROW; r++ )
	{
		Norm = Norm + pow(m_Value[r][0],2);
	}
	Norm = sqrt(Norm);
	return Norm;
}


//============================================================
// Matrix Augmentation
//============================================================
void LinearAlgebra::MatAugment(char s[100], LinearAlgebra& A, LinearAlgebra& B)
{
	int r, c;
	if (s == "row")
	{
		for( r=0; r<A.m_ROW; r++ )
		{
			for( c=0; c<A.m_COL; c++ )
			{
				m_Value[r][c] = A.m_Value[r][c];
			}
		}
		for( r=0; r<B.m_ROW; r++ )
		{
			for( c=0; c<B.m_COL; c++ )
			{
				m_Value[A.m_ROW + r][c] = B.m_Value[r][c];
			}
		}
	}
	else if (s == "col")
	{
		for( r=0; r<A.m_ROW; r++ )
		{
			for( c=0; c<A.m_COL; c++ )
			{
				m_Value[r][c] = A.m_Value[r][c];
			}
		}
		for( r=0; r<B.m_ROW; r++ )
		{
			for( c=0; c<B.m_COL; c++ )
			{
				m_Value[r][A.m_COL + c] = B.m_Value[r][c];
			}
		}
	}
}



//============================================================
// Matrix DeAugmentation
//============================================================
void LinearAlgebra::MatDeAugment(char s[], int A, int B, LinearAlgebra& C)
{
	int r, c;
	if (s == "row")
	{
		if (B == 1)
		{
			for( r=0; r<A; r++ )
			{
				for( c=0; c<C.m_COL; c++ )
				{
					m_Value[r][c] = C.m_Value[r][c];
				}
			}
		}
		else if (B == 2)
		{
			for( r=A; r<C.m_ROW; r++ )
			{
				for( c=0; c<C.m_COL; c++ )
				{
					m_Value[r-A][c] = C.m_Value[r][c];
				}
			}
		}
	}
	else if (s == "col")
	{
		if ( B == 1)
		{
			for( r=0; r<C.m_ROW; r++ )
			{
				for( c=0; c<A; c++ )
				{
					m_Value[r][c] = C.m_Value[r][c];
				}
			}
		}
		else if (B == 2)
		{
			for( r=0; r<C.m_ROW; r++ )
			{
				for( c=A; c<C.m_COL; c++ )
				{
					m_Value[r][c-A] = C.m_Value[r][c];
				}
			}
		}
	}
}



//============================================================
// Zeros Matrix
//============================================================
void LinearAlgebra::MatZeros(int r, int c)
{
	int i, j;
	for( i=0; i<r; i++ )
	{
		for( j=0; j<c; j++ )
		{
			m_Value[i][j] = 0; 
		}
	}
}


//============================================================
// File Write
//============================================================

void LinearAlgebra::FileWrite(FILE *fp, char mode[], char datatitle[])
{
	int r, c;
//	FILE *fp;
//	fp = fopen(filename,mode);
	fprintf(fp,"%s\n",datatitle);

	for( r=0; r<m_ROW; r++ )
	{
		for( c=0; c<m_COL; c++ )
		{
			fprintf(fp,"%f\t",m_Value[r][c]);
		}
		fprintf(fp,"\n");
	}
	fprintf(fp,"\n");
//	fclose(fp);
}
/*
void LinearAlgebra::FileWrite(char filename[], char mode[], char datatitle[])
{
	int r, c;
	FILE *fp;
	fp = fopen(filename,mode);
	fprintf(fp,"%s\n",datatitle);

	for( r=0; r<m_ROW; r++ )
	{
		for( c=0; c<m_COL; c++ )
		{
			fprintf(fp,"%f\t",m_Value[r][c]);
		}
		fprintf(fp,"\n");
	}
	fprintf(fp,"\n");
	fclose(fp);
}
*/

//============================================================
// File Delete
//============================================================
void LinearAlgebra::FileClear(char filename[])
{
	delete(filename);
}
