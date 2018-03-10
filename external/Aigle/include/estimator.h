#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <float.h>

#include "aigle_types.h"

#include "matrix/math.hpp"

#define earth_radius 			6371000.0
#define one_g					9.80665f
#ifndef M_PI
    #define M_PI 				3.14159265358979323846
#endif

#define M_DEG_TO_RAD 			M_PI/180.0
#define EPSILON_FLOAT			1.0e-7f


// Constants for state representation
#define X_qw  0
#define X_qx  1
#define X_qy  2
#define X_qz  3
// #define X_wx  4
// #define X_wy  5
// #define X_wz  6
#define X_bwx  4
#define X_bwy  5
#define X_bwz  6
#define X_x  7
#define X_y  8
#define X_z  9
#define X_vx  10
#define X_vy  11
#define X_vz  12
// #define X_ax  16
// #define X_ay  17
// #define X_az  18
#define X_bax  13
#define X_bay  14
#define X_baz  15
#define n_X  16
// #define n_X 7

// Constants for Inputs variable
#define U_wx  0
#define U_wy  1
#define U_wz  2
#define U_bwx  3
#define U_bwy  4
#define U_bwz  5
#define U_ax  6
#define U_ay  7
#define U_az  8
#define U_bax  9
#define U_bay  10
#define U_baz  11
#define n_U  12

// Constants for Measurement variable
#define Y_gps_x  0
#define Y_gps_y  1
#define Y_gps_z  2
#define Y_gps_yaw  3
#define n_Y  4


const matrix::Vector3f gravity(0.0 , 0.0 , -one_g);

class Estimator
{

public:

	// Constants for process noise and measurement noise variance
	static constexpr float sigma_w = 0.0003394;// 1.5e-2;
	// static const float sigma_wy = 1.5e-2;
	// static const float sigma_wz = 1.5e-2;
	static constexpr float sigma_bw = 3.8785e-05;// 1e-3;
	// static const float sigma_bwy = 1e-3;
	// static const float sigma_bwz = 1e-3;
	static constexpr float sigma_a = 0.004; // 3.5e-1;
	// static const float sigma_ay = 3.5e-1;
	// static const float sigma_az = 3.5e-1;
	static constexpr float sigma_ba = 0.0006;// 3.0e-3;
	// static const float sigma_bay = 3.0e-3;
	// static const float sigma_baz = 3.0e-3;
	static constexpr float sigma_mag = 1e-2;
	static constexpr float sigma_gps_x = 0.01;
	static constexpr float sigma_gps_y = 0.01;
	static constexpr float sigma_gps_z = 0.01;
	static constexpr float sigma_gps_yaw = 5e-3;

	static constexpr float corr_time_acc = 3.0e-3f;
	static constexpr float corr_time_gyro = 1.0e-3f;


	static const uint64_t mag_update_time = 100; //ms

	static const uint16_t max_init_sample = 1000;

	Estimator();

	void update();

	void setGPSQueue(QueueHandle_t gpsQueueHandle);
	void setIMUQueue(QueueHandle_t imuQueueHandle);
	void setEstimatePosQueue(QueueHandle_t estimQueue);

	~Estimator();

private:

	// state space
	matrix::Vector<float, n_X> _x; 				// State vector
	matrix::Vector<float, n_U>	_u; 			// Input vector
	matrix::Matrix<float, n_X , n_X> _P;		// state covariance matrix
	matrix::Matrix<float, n_U , n_U> _Q;		// process noise matrix

	matrix::Matrix<float, n_X , n_X> _Fx;  		// Gradient matrix for state function
	matrix::Matrix<float, n_X , n_U> _Fu;		// Gradient matrix for input function
	matrix::Dcm<float> _Rbn;					// Direct cosinux matrix
	matrix::Vector3f  _a_next; 					// Not include Rbn transformation
	matrix::Vector3f a_next;					// Include true Rbn and gravity
	matrix::Vector3f w_next;
	matrix::Vector3f _w_dt;
	matrix::Quatf _q_w_dt;

	void predict(float dt);
	matrix::Vector<float , n_X> model_dynamic(float dt );

	void initSystem(float currentTimeStamp);

	void gpsCorrect(const gps_data *m_pos);
	void yawCorrect(const matrix::Vector3f &mag_data);
	void imuCorrect(const matrix::Vector3f &acc_data);

	void project(double lat , double lon , float *x , float *y);

	uint64_t _timeStamp;
	uint16_t _sample_obtained;

	uint64_t _mag_timeout;
	// uint8_t _first_update;
	// uint64_t _firstTimeUpdate;

	// Queue to receive gps and imu data
	QueueHandle_t _gpsQueueHandle;
	QueueHandle_t _imuQueueHandle;
	QueueHandle_t _estimQueue;

	// Initial mean of gravity , magnetometer and gyro bias estimate 
	matrix::Vector3f init_g_b;				// gravity in body frame
	matrix::Vector3f init_mag_b;			// Magnetometer vector 
	matrix::Vector3f init_gyro_bias_b;		// Gyro bias in body frame

	// Initial gps --> lat and lon are in radians
	uint32_t gpsDataCount;
	double initgpslat ;
	double initgpslon ;
	float initgpsalt;
	double cos_initgpslat;
	double sin_initgpslat;
};

#endif //ESTIMATOR_H