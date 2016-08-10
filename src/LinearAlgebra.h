//=================================================================================
// NOTE: It is completly up to the user to make sure that all the matrix dimesions
// agree during any matrix opoeration.  There is no dimension mismatch error
// checking in any of these functions.  Your program will crash if the dimesions
// are wrong, or may return invalid results.
//=================================================================================
// m_Value is the matrix that holds the values of the matrix.  Note that a vector is a special 
// case of a matrix where one of the dimesions is 1.  To access the data simply use m_Value[i][j],
// where i and j start at 0. If you are using a vector then either i or j needs to be 1 
// depending if it is a row or column vector.
//=================================================================================
// To use these functions the LEFT hand side of the operator must always be of type LinearAlgebra
// NOTE: all the functions return double**
//
// CANNOT do this:	D = A*B + C
// --> A*B returns type double** not LinearAlgebra which means that it is not an object,
//		and thus cannot call the + operator
//
// CAN do this:	D = C + A*B
// --> C is of type LinearAlgebra, and + is defined for a right-hand operand of type double**
//
// This is the only way to do the math routines with out having the overhead of creating memory
// for temporary LinearAlgebra objects so that the functions can return type LinearAlgebra
// rather than type double**
//
// Also there is no function definition for * that takes a double** as the right operand.  This is
// because there would be no way to know the dimensions of the double** matrix, and thus due to the
// way the * function works it could not properly calculate a matrix multiply.
//=================================================================================
// The inverse can be taken of any matrix.  It does not need to be square.  If the matrix is not
// square than a psuedo inverse is done via the SVD.
//=================================================================================
#pragma once

//These are functions used in SVD calcualtion
#define SIGN(a,b) ( (b) > 0.0 ? fabs(a) : -fabs(a) )
static double sqrarg;
#define SQR(a) ( (sqrarg=(a)) == 0.0 ? 0.0 : sqrarg*sqrarg)
static double dmaxarg1, dmaxarg2;
#define DMAX(a,b) ( dmaxarg1=(a),dmaxarg2=(b), (dmaxarg1) > (dmaxarg2) ? (dmaxarg1) : (dmaxarg2) )
static double iminarg1, iminarg2;
#define IMIN(a,b) ( iminarg1=(a),iminarg2=(b), (iminarg1) < (iminarg2) ? (iminarg1) : (iminarg2) )


class LinearAlgebra
{

public:
	//-----VARIABLES-----
	double** m_Value;	

	//-----FUNCTIONS-----
	LinearAlgebra(void);	//constructor
	~LinearAlgebra(void);	//destructor

	void		CreateMemory(int numOfRows, int numOfCol);	//Use this to create memory for matrix
	void		DeleteMemory(void);
	void		Inverse		(void);		
	int			Row			(void);				//returns the number of rows in the matrix
	int			Col			(void);				//returns the number of columns in the matrix
	
	void		operator = (LinearAlgebra& B);
	void		operator = (double** PtrValue);

	double**	operator + (LinearAlgebra& B);
	double**	operator + (double** B);
	double**	operator + (double Scalar);

	double**	operator - (LinearAlgebra& B);
	double**	operator - (double** B);
	double**	operator - (double Scalar);
	double**	operator - (void);				//negative of a matrix

	double**	operator * (LinearAlgebra& B);
	double**	operator * (double Scalar);

	double**	operator ^ (int Pwr);			// (-1 is inverse) and (1 is transpose)

	void	MatMult(LinearAlgebra& A, LinearAlgebra& B, LinearAlgebra& C);
	void	MatDisplay(char s[]);
	void	MatTranspose(LinearAlgebra& A);
	double	MatDeterminant(void);
	void	MatFileWrite(void);
	double	MatTrace(void);
	void	MatSkew(LinearAlgebra &A);
	void	MatAntiSkew(LinearAlgebra &A);
	void	MatScale(double ScaleFactor);
	void	Mat2Vector(int A, LinearAlgebra& B);
	void	MatAugment(char s[], LinearAlgebra& A, LinearAlgebra& B);
	void	MatDeAugment(char s[], int A, int B, LinearAlgebra& C);
	void	MatZeros(int r, int c);
	double	VectorNorm(void);
	double	GetSinc(double Angle);
	int		GetNumElements (void);			//returns the number of elements in the matrix
	void	FileWrite(FILE* fp, char mode[], char datatitle[]);
	//void	FileWrite(char filename[], char mode[], char datatitle[]);
	void	FileClear(char filename[]);

	double** m_InverseVal;

// private:
	//-----VARIABLES-----
	int m_ROW;
	int m_COL;

	//temp matrices for calculations
	double** m_TempVal;
	double** m_TempVal2;
	double** m_TempValnxn;
    double** m_TransposeVal;
	
	//matrices and vectors used in SVD calculation
	double** m_U;
	double*  m_W;
	double** m_V;
	double*	 m_rv1;

	//-----FUNCTIONS-----

	void	SVD			(void);
	double	Pythag		(double a, double b);
};
