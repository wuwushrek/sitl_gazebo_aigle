#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cstdint>
#include <string>

#include <Matrix.hpp>

using namespace matrix;

// Constants for state representation
#define X_qx  0;
#define X_qy  1;
#define X_qz  2;
#define X_qw  3;
#define X_wx  4;
#define X_wy  5;
#define X_wz  6;
#define X_bwx  7;
#define X_bwy  8;
#define X_bwz  9;
#define X_x  10;
#define X_y  11;
#define X_z  12;
#define X_vx  13;
#define X_vy  14;
#define X_vz  15;
#define X_ax  16;
#define X_ay  17;
#define X_az  18;
#define X_bax  19;
#define X_bay  20;
#define X_baz  21;
#define n_X  22;

// Constants for Inputs variable
#define U_wx  0;
#define U_wy  1;
#define U_wz  2;
#define U_ax  3;
#define U_ay  4;
#define U_az  5;
#define n_U  6;

// Constants for Measurement variable
#define Y_gps_x  0;
#define Y_gps_y  1;
#define Y_gps_z  2;
#define Y_gps_yaw  3;
#define n_Y  4;



class Estimator
{

public:

	// Constants for process noise and measurement noise variance
	static const float sigma_wx = 1.5e-2;
	static const float sigma_wy = 1.5e-2;
	static const float sigma_wz = 1.5e-2;
	static const float sigma_bwx = 1e-3;
	static const float sigma_bwy = 1e-3;
	static const float sigma_bwz = 1e-3;
	static const float sigma_ax = 3.5e-1;
	static const float sigma_ay = 3.5e-1;
	static const float sigma_az = 3.5e-1;
	static const float sigma_bax = 3.0e-3;
	static const float sigma_bay = 3.0e-3;
	static const float sigma_baz = 3.0e-3;
	static const float sigma_gps_x = 0.5;
	static const float sigma_gps_y = 0.5;
	static const float sigma_gps_z = 0.5;
	static const float sigma_gps_yaw = 5e-2;

	static const uint16_t max_init_sample = 10000;

	Estimator();
	void update();
	~Estimator();

private:

	// state space
	Vector<float, n_X> _x; 				// State vector
	Vector<float, n_U>	_u; 			// Input vector
	Matrix<float, n_X , n_X> _P;		// state covariance matrix
	Matrix<float, n_U*2 , n_U*2> _Q;	// process noise matrix

	Matrix<float, n_X , n_X> _Fx;  		// Gradient matrix for state function
	Matrix<float, n_U , n_U> _Fu;		// Gradient matrix for input function

	void predict();
	Vector<float , n_X> model_dynamic(float dt , const Vector<float, n_X> &x, const Vector<float , n_U> &u);

	void initX();
	void initP();
	void initQ();

	void gpsInit();
	void gpsCorrect();

	void imuInit();
	void imuCorrect();

	uint64_t _timeStamp;
	uint16_t _sample_obtained;

};

#endif //ESTIMATOR_H