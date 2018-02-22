#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cstdint>
#include <string>

#include <Matrix.hpp>

using namespace matrix;



class Estimator
{

public:
	// Constants for state representation
	static const uint8_t X_qx = 0;
	static const uint8_t X_qy = 1;
	static const uint8_t X_qz = 2;
	static const uint8_t X_qw = 3;
	static const uint8_t X_wx = 4;
	static const uint8_t X_wy = 5;
	static const uint8_t X_wz = 6;
	static const uint8_t X_bwx = 7;
	static const uint8_t X_bwy = 8;
	static const uint8_t X_bwz = 9;
	static const uint8_t X_x = 10;
	static const uint8_t X_y = 11;
	static const uint8_t X_z = 12;
	static const uint8_t X_vx = 13;
	static const uint8_t X_vy = 14;
	static const uint8_t X_vz = 15;
	static const uint8_t X_ax = 16;
	static const uint8_t X_ay = 17;
	static const uint8_t X_az = 18;
	static const uint8_t X_bax = 19;
	static const uint8_t X_bay = 20;
	static const uint8_t X_baz = 21;
	static const uint8_t n_X = 22;

	// Constants for Inputs variable
	static const uint8_t U_wx = 0;
	static const uint8_t U_wy = 1;
	static const uint8_t U_wz = 2;
	static const uint8_t U_ax = 3;
	static const uint8_t U_ay = 4;
	static const uint8_t U_az = 5;
	static const uint8_t n_U = 6;

	// Constants for Measurement variable
	static const uint8_t Y_gps_x = 0;
	static const uint8_t Y_gps_y = 1;
	static const uint8_t Y_gps_z = 2;
	static const uint8_t Y_gps_yaw = 3;
	static const uint8_t n_Y = 4;

	Estimator();
	void update();
	~Estimator();

private:

	// state space
	Vector<float, n_X> _x; // State vector
	Vector<float, n_U>	_u; // Input vector
	Matrix<float, n_X , n_X> _P;	//state covariance matrix

	Matrix<float, n_X , n_X> _Fx;  	// Gradient matrix for state function
	Matrix<float, n_U , n_U> _Fu;	// Gradient matrix for input function

	void predict();
	Vector<float , n_X> model_dynamic(float dt , const Vector<float, n_X> &x, const Vector<float , n_U> &u);

	void initState();
	
	void gpsInit();
	void gpsCorrect();

	void imuInit();
	void imuCorrect();


	// Update status
	bool _gpsUpdated;

	// last time gps received
	uint64_t _time_last_gps;

	// reference altitudes

};

#endif //ESTIMATOR_H