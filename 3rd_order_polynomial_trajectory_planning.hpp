#ifndef __3POLYNOMIAL__
#define __3POLYNOMIAL__

#define TTT(x) (x*x*x)
#define TT(x) (x*x)

#include <cmath>

struct kinematics
{
	double position;
	double velocity;
	double acceleration;
};



kinematics cubic(double time, double* coeff);
void coefficient(double start_position, double final_position, double final_time,double* coeff);

#endif __3POLYNOMIAL__
