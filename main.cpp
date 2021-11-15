#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <cstdio>
#include "3rd_order_polynomial_trajectory_planning.hpp"
#include "trapezoid_accelerationANDdeceleration.hpp"

using namespace std;

int main(int argc, char** argv)
{
	/* 1) 3rd order polynomial trajectory planning */
	//FILE* fp = fopen("3rd order polynomial.txt", "w");
	//int start_position = 0, final_position = 1000;
	//double final_time = 1.0, start_time = 0.001, time = 0;
	//double coeff[4];
	//coefficient(start_position, final_position, final_time, coeff);

	//kinematics motion[1000];
	//int tmp = 0;
	//cout << "\tDifferential\tPosition\tVelocity\tAcceleration\n\n";
	//fprintf(fp, "Differential\tPosition\t\tVelocity\t\tAcceleration\n");
	//while (time < final_time)
	//{
	//	motion[tmp] = cubic(time, coeff);
	//	printf("\t    %d\t\t%.3f\t\t%.3f\t\t%.3f\n", tmp, motion[tmp].position, motion[tmp].velocity, motion[tmp].acceleration);
	//	fprintf(fp, "%d\t\t%.3f\t\t%.3f\t\t%.3f\n", tmp, motion[tmp].position, motion[tmp].velocity, motion[tmp].acceleration);
	//	tmp += 1;
	//	time = start_time * tmp;
	//}
	//fclose(fp);

	/* 2) Trapezoidal acceleration and deceleration*/
	FILE* fp = fopen("Trapezoidal.txt", "w");
	double start_position = 0.0, start_velocity = 0.0, final_position = 1500, final_velocity = 0.0, max_velocity = 1000, max_acceleration = 1000;
	double accel_time = 0, constant_time = 0, final_time, time_section = 0.001;
	int tmp = 0;
	motion m[3000];
	trapezoid_coeff(start_position, final_position, max_velocity, max_acceleration, accel_time, constant_time, final_time);
	double current_time = 0.0;
	cout << "Section\t\tPosition\tVelocity\tAcceleration\n\n";
	fprintf(fp, "Section  Position\tVelocity\tAcceleration\n");
	while (current_time <= final_time)
	{
		m[tmp] = trapezoidar(start_position, final_position, max_velocity, max_acceleration, accel_time, final_time, constant_time, current_time);
		fprintf(stdout, "%d\t\t%f\t%f\t%f\n", tmp, m[tmp].position, m[tmp].velocity, m[tmp].acceleration);
		fprintf(fp, "%d\t%f\t%f\t%f\n", tmp, m[tmp].position, m[tmp].velocity, m[tmp].acceleration);
		tmp += 1;
		current_time = tmp * time_section;
	}
	fclose(fp);
	return 0;
}