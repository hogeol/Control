#include "3rd_order_polynomial_trajectory_planning.hpp"

kinematics cubic(double time, double* coeff)
{
	kinematics pva;
	pva.position = coeff[3] * TTT(time) + coeff[2] * TT(time) + coeff[1] * time + coeff[0];
	pva.velocity = 3 * coeff[3] * TT(time) + 2 * coeff[2] * time + coeff[1];
	pva.acceleration = 6 * coeff[3] * time + 2 * coeff[2];

	return pva;
}

void coefficient(double start_position, double final_position, double final_time, double* coeff)
{
	coeff[0] = start_position;
	coeff[1] = 0;
	coeff[2] = 3.0 * (final_position - start_position) / (TT(final_time));
	coeff[3] = -2.0 * (final_position - start_position) / (TTT(final_time));
}
