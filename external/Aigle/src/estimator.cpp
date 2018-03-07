
#include "estimator.h"

using namespace matrix;

Matrix<float , 4 , 3> euler2QuatJacobian(float roll , float pitch , float yaw);

Estimator::Estimator(){

	// init _Fx and _Fu
	_Fx.setZero();
	_Fu.setZero();
	_Q.setZero();
	_u.setZero();
	for (uint8_t i =0 ; i< 3 ; i++){
		_Q(U_wx + i , U_wx + i) = sigma_w * sigma_w;
		_Q(U_bwx + i ,  U_bwx + i ) = sigma_bw * sigma_bw;
		_Q(U_ax + i , U_ax + i) = sigma_a * sigma_a;
		_Q(U_bax + i ,  U_bax + i ) = sigma_ba * sigma_ba;
	}

	// Init gps origin
	gpsDataCount = 0;
	initgpslat = 0.0;
	initgpslon = 0.0;
	initgpsalt = 0.0f;

	// Initial timeStamp constant
	_timeStamp = 0;
	_sample_obtained = 0;
	_mag_timeout = 0;

	// initial bias, acc and mag 
	init_g_b.setZero();
	init_mag_b.setZero();
	init_gyro_bias_b.setZero();

	// _first_update = 1;
}

Estimator::~Estimator(){

}

void Estimator::setGPSQueue(QueueHandle_t gpsQueueHandle){
	_gpsQueueHandle = gpsQueueHandle;
}

void Estimator::setIMUQueue(QueueHandle_t imuQueueHandle){
	_imuQueueHandle = imuQueueHandle;
}

void Estimator::setEstimatePosQueue(QueueHandle_t estimQueue){
	_estimQueue = estimQueue;
}

void Estimator::update(){
	// Updated values from IMU sensor
	imu_data last_imu;

	// Check if there is any updated IMU data with 50ms as max blocking time
	if ( ! xQueueReceive(_imuQueueHandle , &last_imu , pdMS_TO_TICKS(50))){
		return ;
	}

	// get current time and deltat
	uint64_t currentTimeStamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
	float dt = (currentTimeStamp - _timeStamp)/1.0e3f;
	_mag_timeout += (currentTimeStamp - _timeStamp);
	_timeStamp = currentTimeStamp;

	// Still gathering data for initial estimation
	if (_sample_obtained < max_init_sample){
		// We calculate an riemann sum while max_init_sample isn't reached
		init_g_b += Vector3f(last_imu.xacc * dt , last_imu.yacc * dt , last_imu.zacc * dt) ;  
		init_mag_b += Vector3f(last_imu.xmag * dt , last_imu.ymag * dt , last_imu.zmag * dt);
		init_gyro_bias_b += Vector3f(last_imu.xgyro * dt , last_imu.ygyro * dt , last_imu.zgyro * dt);

		// sum of all gps alt lat and lon
		gps_data gps;
		if ( xQueueReceive(_gpsQueueHandle , &gps , ( TickType_t ) 0 ) ){
			gpsDataCount+=1;
			initgpsalt += (gps.alt * 1.0e-3);
			initgpslat += (gps.lat * 1.0e-7);
			initgpslon += (gps.lon * 1.0e-7);
		} 

		// Increment number of imu data taken
		_sample_obtained+=1;

		// If sample is complete --> divide Riemann sum with time elapsed for obtaining a 'mean'
		if (_sample_obtained >= max_init_sample){
			float time_sec = currentTimeStamp / 1.0e3f;
			init_g_b /= time_sec ;
			init_mag_b /= time_sec;
			init_gyro_bias_b /= time_sec;

			// Set lat and lon in radian
			initgpslon /= (gpsDataCount); /// M_DEG_TO_RAD);
			initgpslat /= (gpsDataCount); /// M_DEG_TO_RAD);
			initgpslon *= M_DEG_TO_RAD;
			initgpslat *= M_DEG_TO_RAD;
			initgpsalt /= gpsDataCount ;
			cos_initgpslat = cos( initgpslat );
			sin_initgpslat = sin( initgpslat );
			initSystem(time_sec);
			// std::cout << "New x : " << std::endl;
			// std::cout << _x << std::endl;
		}
		return ;
	}

	_u(U_ax) = last_imu.xacc;
	_u(U_ay) = last_imu.yacc;
	_u(U_az) = last_imu.zacc;
	_u(U_wx) = last_imu.xgyro;
	_u(U_wy) = last_imu.ygyro;
	_u(U_wz) = last_imu.zgyro;

	_x =  model_dynamic(dt);
	predict(dt);

	if(_mag_timeout > mag_update_time){
		Vector3f m_mag (last_imu.xmag ,  last_imu.ymag ,  last_imu.zmag);
		yawCorrect(m_mag);
		_mag_timeout = 0;
	}

	// if (a_next.norm() < 1e-1 && w_next.norm() < 1e-2) {
	// 	Vector3f acc_mes(last_imu.xacc , last_imu.yacc , last_imu.zacc);
	// 	imuCorrect(acc_mes);
	// }

	gps_data gps;
	if ( xQueueReceive(_gpsQueueHandle , &gps , ( TickType_t ) 0 ) ){
		gpsCorrect(&gps);
	}
	// std::cout << "Delay = " << dt << std::endl;

}

void Estimator::initSystem(float currentTimeStamp){
	// Roll estimate from gravity vector
	float roll_init = atan2(init_g_b(1) , init_g_b(2));
	float pitch_init = atan2(-init_g_b(0), sqrt(init_g_b(1)* init_g_b(1) + init_g_b(2)* init_g_b(2)));

	// Initial yaw estimation
	// float yaw_init = atan2(- init_mag_b(1), init_mag_b(0));
	float yaw_init = atan2(-init_mag_b(1) * cos(roll_init) + init_mag_b(2)*sin(roll_init) , 
		init_mag_b(0)*cos(pitch_init)+init_mag_b(1)*sin(pitch_init)*sin(roll_init)+init_mag_b(2)*sin(pitch_init)*cos(roll_init) );
	Eulerf euler(roll_init, pitch_init , yaw_init);

	// Convert euler to quaternion
	Quatf q_init(euler);

	/* State vector initialization --> Hypth of non moving vehicle */
	_x.setZero();
	// Initialiaze state vector initial quaternion
	_x(X_qw) = q_init(0);
	_x(X_qx) = q_init(1);
	_x(X_qy) = q_init(2);
	_x(X_qz) = q_init(3);

	// Intialize state vector with initial gyro bias estimate
	_x(X_bwx) = init_gyro_bias_b(0);
	_x(X_bwy) = init_gyro_bias_b(1);
	_x(X_bwz) = init_gyro_bias_b(2);

	// Initialize bias with the difference with default gravity (this assume the mobile not move and imu is not inclinated)
	Dcmf initDcmf(q_init);
	Vector3f init_b_a  = init_g_b + initDcmf.transpose() * gravity; 
	std::cout << initDcmf.transpose() * gravity << std::endl; 
	_x(X_bax) = init_b_a(0);
	_x(X_bay) = init_b_a(1);
	_x(X_baz) = init_b_a(2);

	_Rbn = Dcmf(Quatf(_x.data() + X_qw));
	/**************************************************************/

	/* Initialize covariance matrix error 						   */
	_P.setZero();

	// Covariance matrix for euler angle first estimation
	// float sigma_a = max(sigma_ax, max(sigma_ay , sigma_az));
	// float sigma_m = max(sigma_wx, max(sigma_wy , sigma_wz));
	Matrix<float , 3 , 3> euler_error;
	euler_error.setZero();
	euler_error(0,0) = sigma_a * sigma_a / currentTimeStamp;
	euler_error(1,1) = sigma_a * sigma_a / currentTimeStamp; 
	euler_error(2,2) = sigma_mag * sigma_mag / currentTimeStamp;

	// Get covariance matrix for initial attitude
	Matrix<float , 4 , 3> jacob = euler2QuatJacobian(roll_init, pitch_init ,yaw_init);
	Matrix<float , 4 , 4> cov_Q = jacob * euler_error * jacob.transpose();
	// Filling the covariance matrix _P 
	uint8_t i , j;
	// Star by setting diagonal element to small positive values
	for (i = 0; i<n_X ; i++){
		_P(i,i) = EPSILON_FLOAT;
	}
	// set covariance matrix for iniial attitude 
	for (i = 0 ; i< 4 ; i++){
		for (j = 0 ; j< 4 ; j++){
			_P(X_qw+i,X_qw+j) = cov_Q(i,j);
		}
	}
	// Fill _P with uncertaincy from initial gyro measurement
	// for (i = X_wx ; i< X_wx + 3 ; i++){
	// 	_P(i,i) = sigma_w * sigma_w / currentTimeStamp;
	// }
	/***************************************************************/
	std::cout << "Init acc = " << init_g_b << std::endl;
	std::cout << "Init mag = " << init_mag_b << std::endl;
	std::cout << "Init gyr = " << init_gyro_bias_b << std::endl;

	std::cout << "Init roll = " << roll_init << std::endl;
	std::cout << "Init pitch = " << pitch_init << std::endl;
	std::cout << "Init yaw = " << yaw_init << std::endl;

	std::cout << "Init lat = " << initgpslat << std::endl;
	std::cout << "Init lon = " << initgpslon << std::endl; 
	std::cout << "Init alt = " << initgpsalt << std::endl;
	std::cout << "Init time = " << currentTimeStamp << std::endl;

	std::cout << "\nInitial state : " << std::endl;
	std::cout << _x << std::endl;
	// std::cout << "\nInitial cov : " << std::endl;
	// std::cout << _P << std::endl;
}

Vector<float , n_X> Estimator::model_dynamic(float dt ){
	Vector<float , n_X> nextX;
	uint8_t i;

	// _Rbn = Dcmf(Quatf(_x.data() + X_qw));

	// Update gyro estimate with gyro input + biais
	for(i= 0 ; i< 3 ; i++ ){
		// nextX(i+X_wx) = (_u(U_wx+i) - _x(X_bwx+i)); // TODO : - ou pas ??
		nextX(i+X_bwx) = (1.0f - dt*corr_time_gyro) * _x(X_bwx + i);
		w_next(i) = (_u(U_wx+i) - _x(X_bwx+i));
		_w_dt(i) = (_u(U_wx+i) - _x(X_bwx+i)) * dt;
	}

	// update accelero,position and velocity estimate with acc input + biais
	_a_next = Vector3f(_u(U_ax)-_x(X_bax) , _u(U_ay)-_x(X_bay) , _u(U_az)-_x(X_baz)) ;
	a_next = _Rbn * _a_next + gravity;
	for(i=0 ; i<3 ; i++){
		// nextX(i+X_ax) = a_next(i);
		nextX(i+X_bax) = (1.0f - dt*corr_time_acc) * _x(X_bax + i);
		nextX(i+X_vx) = _x(i+X_vx) + a_next(i) * dt;
		nextX(i+X_x) = _x(i+X_x) + nextX(i+X_vx) * dt + 0.5f * a_next(i) * dt * dt;
	}

	// update Quaternion state
	_q_w_dt = Quatf(AxisAnglef(_w_dt));
	Quatf q_next = Quatf(_x.data() + X_qw) * _q_w_dt;
	for(i=0; i<4 ; i++){
		nextX(X_qw+i) = q_next(i);
	}

	_Rbn = Dcmf(Quatf(_x.data() + X_qw));
	// std::cout << "New x : " << std::endl;
	// std::cout << _x(X_x) << std::endl;
	// std::cout << a_next << std::endl;
	// std::cout << w_next << std::endl;
	// std::cout << _x(X_x) << " , " << _x(X_y) << " , " << _x(X_z)  << std::endl;
	return nextX;
}

void Estimator::predict(float dt){
	uint8_t i ,j;
	// quat, quat
	_Fx(X_qw, X_qw) = _q_w_dt(0);
	_Fx(X_qw, X_qx) = -_q_w_dt(1);
	_Fx(X_qw, X_qy) = -_q_w_dt(2);
	_Fx(X_qw, X_qz) = -_q_w_dt(3);

	_Fx(X_qx, X_qw) = _q_w_dt(1);
	_Fx(X_qx, X_qx) = _q_w_dt(0);
	_Fx(X_qx, X_qy) = _q_w_dt(3);
	_Fx(X_qx, X_qz) = -_q_w_dt(2);

	_Fx(X_qy, X_qw) = _q_w_dt(2);
	_Fx(X_qy, X_qx) = -_q_w_dt(3);
	_Fx(X_qy, X_qy) = _q_w_dt(0);
	_Fx(X_qy, X_qz) = _q_w_dt(1);

	_Fx(X_qz, X_qw) = _q_w_dt(3);
	_Fx(X_qz, X_qx) = _q_w_dt(2);
	_Fx(X_qz, X_qy) = -_q_w_dt(1);
	_Fx(X_qz, X_qz) = _q_w_dt(0);

	// quaternion , gyro bias
	float angle = _w_dt.norm();
	Quatf fq_wx, fq_wy,fq_wz;

	if (abs(angle) > 1e-10f){
		float u = cos(angle/2.0f);
		float v = sin(angle/2.0f);
		float v_angle = v/angle; // Only if the angle is not close to zero
		float v_angle_der = (0.5f * u   - v_angle)/ (angle * angle);
		//		Derivative in respect to wbx
		float temp_der_wx = dt * _w_dt(0) * v_angle_der; 
		fq_wx = Quatf(-0.5f * dt * _w_dt(0) * v_angle, temp_der_wx * _w_dt(0) + v_angle*dt , temp_der_wx * _w_dt(1) , temp_der_wx * _w_dt(2) );

		//		Derivative in respect to wby
		float temp_der_wy = dt * _w_dt(1) * v_angle_der; 
		fq_wy = Quatf(-0.5f * dt * _w_dt(1) * v_angle , temp_der_wy * _w_dt(0) , temp_der_wy * _w_dt(1) + v_angle * dt , temp_der_wy * _w_dt(2) );

		//		Derivative in respect to wby
		float temp_der_wz = dt * _w_dt(2) * v_angle_der; 
		fq_wz = Quatf(-0.5f * dt * _w_dt(2) * v_angle , temp_der_wz * _w_dt(0) , temp_der_wz * _w_dt(1), temp_der_wz * _w_dt(2)+ v_angle*dt );
	} else {
		// First order approximation of the calculation on the non zero case mentionned above
		fq_wx = Quatf(-0.25f * dt * _w_dt(0) , 0.5f * dt , 0.0 , 0.0);
		fq_wx = Quatf(-0.25f * dt * _w_dt(1) , 0.0 , 0.5f * dt , 0.0);
		fq_wy = Quatf(-0.25f * dt * _w_dt(2) , 0.0 , 0.0 , 0.5f * dt);
		std::cout << "Null rotation vector " << std::endl;
	}

	Quatf current_quat(_x.data() + X_qw);
	
	fq_wx = current_quat * fq_wx;
	fq_wy = current_quat * fq_wy;
	fq_wz = current_quat * fq_wz;
	for (i=0 ; i< 4 ; i++){
		_Fx(X_qw+i , X_bwx) = - fq_wx(i);
		_Fx(X_qw+i , X_bwy) = - fq_wy(i);
		_Fx(X_qw+i , X_bwz) = - fq_wz(i);
	}

	// gyro bias , gyro bias
	for(i=0 ; i< 3 ; i++){
		_Fx(X_bwx+i,X_bwx+i) = (1.0f - corr_time_gyro*dt);
	}
	// acc bias , acc bias
	for(i=0 ; i< 3 ; i++){
		_Fx(X_bax+i,X_bax+i) = (1.0f - corr_time_acc*dt);
	}

	// Position , q --> velocity , q
	float data_q0[9] = {_x(X_qw) , -_x(X_qz) , _x(X_qy) , _x(X_qz) , _x(X_qw) , -_x(X_qx) , -_x(X_qy) , _x(X_qx) , _x(X_qw)};
	float data_q1[9] = {_x(X_qx) , _x(X_qy) , _x(X_qz) , _x(X_qy) , -_x(X_qx) , -_x(X_qw) , _x(X_qz) , _x(X_qw) , -_x(X_qx)};
	float data_q2[9] = {-_x(X_qy) , _x(X_qx) , _x(X_qw) , _x(X_qx) , _x(X_qy) , _x(X_qz) , -_x(X_qw) , _x(X_qz) , -_x(X_qy)};
	float data_q3[9] = {-_x(X_qz) , -_x(X_qw) , _x(X_qx) , _x(X_qw) , -_x(X_qz) , _x(X_qy) , _x(X_qx) , _x(X_qy) , _x(X_qw)};

	Vector3f _a_next_2 = _a_next *2.0f;
	Vector3f temp1 = Matrix3f(data_q0) * (_a_next_2 );
	Vector3f temp2 = Matrix3f(data_q1) * (_a_next_2 );
	Vector3f temp3 = Matrix3f(data_q2) * (_a_next_2 );
	Vector3f temp4 = Matrix3f(data_q3) * (_a_next_2 );

	for(i=0; i<3 ; i++){
		_Fx(X_vx + i , X_qw) =  temp1(i) * dt;
		_Fx(X_vx + i , X_qx) =  temp2(i) * dt;
		_Fx(X_vx + i , X_qy) =  temp3(i) * dt;
		_Fx(X_vx + i , X_qz) =  temp4(i) * dt;

		_Fx(X_x + i , X_qw) =  _Fx(X_vx + i , X_qw) * dt * 0.5f;
		_Fx(X_x + i , X_qx) =  _Fx(X_vx + i , X_qx) * dt * 0.5f;
		_Fx(X_x + i , X_qy) =  _Fx(X_vx + i , X_qy) * dt * 0.5f;
		_Fx(X_x + i , X_qz) =  _Fx(X_vx + i , X_qz) * dt * 0.5f;
	}

	// Velocity , accel bias
	for(i=0 ; i <3 ; i++){
		for(j=0 ; j<3 ; j++){
			_Fx(X_vx + i , X_bax + j) = - _Rbn(i,j) * dt;
		}
	}

	// position,position -> position, velocity -> velocity, position
	for(i=0; i<3 ; i++){
		_Fx(X_x + i , X_vx + i) = dt;
		_Fx(X_x + i , X_x + i) = 1.0f;
		_Fx(X_vx + i , X_vx + i ) = 1.0f;
	}

	/* Update gradient of U */
	// quaternion , gyroscope measurement
	// for(i=0 ; i< 4 ; i++){
	// 	_Fu(X_qw+i , U_wx) =  fq_wx(i); // need dt because  q(R(wdt))
	// 	_Fu(X_qw+i , U_wy) =  fq_wy(i);
	// 	_Fu(X_qw+i , U_wz) =  fq_wz(i);
	// }
	// // velocity , acc measurement --> position , acc measurement
	// for(i=0 ; i< 3 ; i++){
	// 	for(j=0 ; j<3 ; j++){
	// 		_Fu(X_vx + i , U_ax + j) = _Rbn(i,j) * dt;
	// 		_Fu(X_x + i , U_ax + j ) = _Fu(X_vx + i , U_ax + j) * 0.5 * dt;
	// 	}
	// }
	// // bias , bias
	// for(i=0 ; i<3 ; i++){
	// 	_Fu(X_bwx + i , U_bwx + i) = 1.0f;
	// 	_Fu(X_bax + i , U_bax + i) =  1.0f;
	// }
	for(i=0 ; i< 4 ; i++){
		_Fu(X_qw+i , U_wx) =  1.0; // need dt because  q(R(wdt))
		_Fu(X_qw+i , U_wy) =  1.0;
		_Fu(X_qw+i , U_wz) =  1.0;
	}
	_Fu(X_qw , U_wx) = 0;
	_Fu(X_qw , U_wy) = 0;
	_Fu(X_qw , U_wz) = 0;
	_Fu(X_qx , U_wx) = 0.5f;
	_Fu(X_qx , U_wy) = 0;
	_Fu(X_qx , U_wz) = 0;
	_Fu(X_qy , U_wx) = 0;
	_Fu(X_qy , U_wy) = 0.5f;
	_Fu(X_qy , U_wz) = 0;
	_Fu(X_qz , U_wx) = 0;
	_Fu(X_qz , U_wy) = 0;
	_Fu(X_qz , U_wz) = 0.5f;
	// velocity , acc measurement --> position , acc measurement
	for(i=0 ; i< 3 ; i++){
		_Fu(X_vx + i , U_ax + i) = 1.0f;
		// _Fu(X_x + i , U_ax + i ) = 1.0f;
	}
	// bias , bias
	for(i=0 ; i<3 ; i++){
		_Fu(X_bwx + i , U_bwx + i) = 1.0f;
		_Fu(X_bax + i , U_bax + i) =  1.0f;
	}
	for (uint8_t i =0 ; i< 3 ; i++){
		_Q(U_wx + i , U_wx + i) = sigma_w * sigma_w * dt * dt;
		_Q(U_bwx + i ,  U_bwx + i ) = sigma_bw * sigma_bw * dt;
		_Q(U_ax + i , U_ax + i) = sigma_a * sigma_a * dt * dt;
		_Q(U_bax + i ,  U_bax + i ) = sigma_ba * sigma_ba * dt;
	}
	// Update error coviarance matrix
	_P = _Fx * _P * _Fx.transpose() + _Fu * _Q * _Fu.transpose();

	if (abs(_x(X_x)) >0.1){
		// std::cout << "Diag : ";
		// for (i=0 ; i< n_X ; i++){
		// 	std::cout << _Fx(i,i) << " ; ";
		// }
		// std::cout << std::endl;
		// std::cout << "Fx print : " << std::endl;
		// std::cout << _Fx << std::endl;
		// std::cout << "Fu print : " << std::endl;
		// std::cout << _Fu << std::endl;
		// std::cout << "P print :" << std::endl;
		// std::cout << _P << std::endl;
	}
}

void Estimator::gpsCorrect(const gps_data *m_pos){
	/* Update position from position measurement */
	float x , y ;
	float z = (m_pos->alt * 1e-3) - initgpsalt;

	// project(m_pos->lat * 1e-7 ,m_pos->lon * 1e-7, &x , &y);
	project(m_pos->lat * 1e-7 ,m_pos->lon * 1e-7, &y , &x); // Because ENU coordinate system
	// h jacobian matrix

	estimate_position estimPos;
	estimPos = {0};
	estimPos.x = x;
	estimPos.y = y;
	estimPos.z = z;
	xQueueOverwrite(_estimQueue, (void * ) &estimPos);
	// std::cout << estimPos.x << estimPos.y << estimPos.z << std::endl;

	Matrix<float,3, n_X> jacob_h;
	jacob_h.setZero();
	jacob_h(0, X_x) = 1.0;
	jacob_h(1, X_y) = 1.0;
	jacob_h(2, X_z) = 1.0;
	Matrix<float , n_X , 3> jacob_h_transpose = jacob_h.transpose();

	// R measurement noise error
	SquareMatrix<float,3> R;
	R.setZero();
	R(0,0) = sigma_gps_x * sigma_gps_x;
	R(1,1) = sigma_gps_y * sigma_gps_y;
	R(2,2) = sigma_gps_z * sigma_gps_z;

	Matrix3f innov_cov = jacob_h * _P * jacob_h_transpose + R;
	Matrix<float , n_X , 3> kalman_gain = _P * jacob_h_transpose * inv(innov_cov);


	Vector3f innov( x - _x(X_x) , y - _x(X_y) , z - _x(X_z) );
	// std::cout << " Innov x,y,z : " << std::endl;
	// std::cout << innov << std::endl;
	// std:: cout << x << " , " << y << " , " << z << std::endl;
	// std::cout << kalman_gain * innov << std::endl;

	// std::cout << _x(X_x) << " , " << _x(X_y) << " , " << _x(X_z)  << std::endl;
	_x += kalman_gain * innov;
	_P -= kalman_gain * innov_cov * kalman_gain.transpose();
	_Rbn = Dcmf(Quatf(_x.data() + X_qw));
	// std:: cout << x << " , " << y << " , " << z << std::endl;
	std::cout << _x(X_x) << " , " << _x(X_y) << " , " << _x(X_z)  << std::endl;
	std::cout << "Diag : ";
	for (uint8_t i=0 ; i< n_X ; i++){
		std::cout << _P(i,i) << " ; ";
	}
	std::cout << std::endl;
	//std::cout << Eulerf(_Rbn) << std::endl;
}

void Estimator::yawCorrect(const matrix::Vector3f &mag_data){
	float num_atan = 2*(_x(X_qx)*_x(X_qy) - _x(X_qw)*_x(X_qz));
	float denom_atan = 1.0f - 2*(_x(X_qy)*_x(X_qy) + _x(X_qz)*_x(X_qz));
	float atan_der_term = 1.0f / (1.0f + ((num_atan * num_atan) /(denom_atan * denom_atan)));
	
	Matrix<float , 1 , n_X> jacob_h_yaw;
	jacob_h_yaw.setZero();
	jacob_h_yaw(0, X_qw) = (-2 * _x(X_qz) / denom_atan) * atan_der_term;
	jacob_h_yaw(0, X_qx) = (2 * _x(X_qy) / denom_atan ) * atan_der_term;
	jacob_h_yaw(0, X_qy) = ((2*_x(X_qx)*denom_atan - (-4 * _x(X_qy))*num_atan)/(denom_atan * denom_atan))* atan_der_term;
	jacob_h_yaw(0, X_qz) = ((-2*_x(X_qw)*denom_atan - (-4 * _x(X_qz))*num_atan)/(denom_atan * denom_atan))* atan_der_term;

	Vector3f mag_data_n = _Rbn * mag_data;
	mag_data_n(2) = 0.0;
	mag_data_n =  _Rbn.transpose() * mag_data_n;

	float yaw_mag = atan2(-mag_data_n(1) , mag_data_n(0));
	float yaw_est = atan2(num_atan, denom_atan);
	float yaw_innov =  yaw_mag - yaw_est;
	float innov_cov_yaw =  (jacob_h_yaw * _P * jacob_h_yaw.transpose())(0,0) + sigma_mag * sigma_mag;
	Matrix<float , n_X , 1> kalman_gain_yaw = (_P * jacob_h_yaw.transpose()) / innov_cov_yaw;
	// std::cout << " Innov yaw : " << std::endl;
	// std::cout << yaw_innov << std::endl;
	// std::cout << Eulerf(_Rbn) << std::endl;
	// std::cout << kalman_gain_yaw * yaw_innov << std::endl;
	// if (denom_atan < 0.1 ){	
	// 	std::cout << "denom_atan " << denom_atan << std::endl;
	// 	std::cout << jacob_h_yaw << std::endl;
	// }
	// if (innov_cov_yaw < 0.01)
	// 	std::cout << "innov cov " << innov_cov_yaw << std::endl;
	// std::cout << _x(X_x) << " , " << _x(X_y) << " , " << _x(X_z)  << std::endl;
	_x += kalman_gain_yaw * yaw_innov;
	_P -= kalman_gain_yaw * (innov_cov_yaw * kalman_gain_yaw.transpose());
	_Rbn = Dcmf(Quatf(_x.data() + X_qw));
	// std::cout << _x(X_x) << " , " << _x(X_y) << " , " << _x(X_z)  << std::endl;
}

void Estimator::imuCorrect(const matrix::Vector3f &acc_data){
	Vector3f gravity_est = _Rbn.transpose() * (-gravity) + Vector3f(_x.data() + X_bax);
	Vector3f innov = acc_data - gravity_est;

	SquareMatrix<float, 3> R;
	R.setZero();
	R(0,0) = sigma_a;
	R(1,1) = sigma_a; 
	R(2,2) = sigma_a;  

	Matrix<float, 3 , n_X> jacob_h;
	jacob_h.setZero();
	float data_q0[9] = {_x(X_qw) , -_x(X_qz) , _x(X_qy) , _x(X_qz) , _x(X_qw) , -_x(X_qx) , -_x(X_qy) , _x(X_qx) , _x(X_qw)};
	float data_q1[9] = {_x(X_qx) , _x(X_qy) , _x(X_qz) , _x(X_qy) , -_x(X_qx) , -_x(X_qw) , _x(X_qz) , _x(X_qw) , -_x(X_qx)};
	float data_q2[9] = {-_x(X_qy) , _x(X_qx) , _x(X_qw) , _x(X_qx) , _x(X_qy) , _x(X_qz) , -_x(X_qw) , _x(X_qz) , -_x(X_qy)};
	float data_q3[9] = {-_x(X_qz) , -_x(X_qw) , _x(X_qx) , _x(X_qw) , -_x(X_qz) , _x(X_qy) , _x(X_qx) , _x(X_qy) , _x(X_qw)};

	Vector3f temp1 = Matrix3f(data_q0).transpose() * (gravity_est );
	Vector3f temp2 = Matrix3f(data_q1).transpose() * (gravity_est );
	Vector3f temp3 = Matrix3f(data_q2).transpose() * (gravity_est );
	Vector3f temp4 = Matrix3f(data_q3).transpose() * (gravity_est );
	for(uint8_t i=0; i<3 ; i++){
		jacob_h(i , X_qw) =  2* temp1(i);
		jacob_h(i , X_qx) =  2* temp2(i);
		jacob_h(i , X_qy) =  2* temp3(i);
		jacob_h(i , X_qz) =  2* temp4(i);
	}
	Matrix3f innov_cov = jacob_h * _P * jacob_h.transpose() + R;
	Matrix<float , n_X , 3> kalman_gain = _P * jacob_h.transpose() * inv(innov_cov);
	_x += kalman_gain * innov;
	_P -= kalman_gain *  innov_cov * kalman_gain.transpose();
	_Rbn = Dcmf(Quatf(_x.data() + X_qw));
	// std::cout << "Innov pitch roll" << std::endl;
	// std::cout << innov << std::endl;

	// std::cout << _x(X_x) << " , " << _x(X_y) << " , " << _x(X_z)  << std::endl;
}

void Estimator::project(double lat , double lon , float *x , float *y){
	double lat_rad = lat * M_DEG_TO_RAD;
	double lon_rad = lon * M_DEG_TO_RAD;

	double sin_lat = sin(lat_rad);
	double cos_lat = cos(lat_rad);
	double cos_d_lon = cos(lon_rad - initgpslon);

	double arg = sin_initgpslat * sin_lat + cos_initgpslat * cos_lat * cos_d_lon;

	if (arg > 1.0) {
		arg = 1.0;

	} else if (arg < -1.0) {
		arg = -1.0;
	}

	double c = acos(arg);
	double k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));

	*x = k * (cos_initgpslat * sin_lat - sin_initgpslat * cos_lat * cos_d_lon) * earth_radius;
	*y = k * cos_lat * sin(lon_rad - initgpslon) * earth_radius;
}

Matrix<float , 4 , 3> euler2QuatJacobian(float roll , float pitch , float yaw){
	float cos_roll = cos(roll * 0.5f);
	float sin_roll = sin(roll * 0.5f);
	float cos_pitch = cos(pitch * 0.5f);
	float sin_pitch = sin(pitch * 0.5f);
	float cos_yaw = cos(yaw * 0.5f);
	float sin_yaw = sin(yaw * 0.5f);

	Matrix<float , 4 , 3> jacob;

	// Roll derivatives
	jacob(0,0) = -0.5f * sin_roll * cos_pitch * cos_yaw + 0.5f * cos_roll * sin_pitch * sin_yaw;
	jacob(1,0) =  0.5f * cos_roll * cos_pitch * cos_yaw + 0.5f * sin_roll * sin_pitch * sin_yaw;
	jacob(2,0) = -0.5f * sin_roll * sin_pitch * cos_yaw + 0.5f * cos_roll * cos_pitch * sin_yaw;
	jacob(3,0) = -0.5f * sin_roll * cos_pitch * sin_yaw - 0.5f * cos_roll * sin_pitch * cos_yaw;

	// Pitch derivatives
	jacob(0,1) = -0.5f * cos_roll * sin_pitch * cos_yaw + 0.5f * sin_roll * cos_pitch * sin_yaw;
	jacob(1,1) = -0.5f * sin_roll * sin_pitch * cos_yaw - 0.5f * cos_roll * cos_pitch * sin_yaw;
	jacob(2,1) =  0.5f * cos_roll * cos_pitch * cos_yaw - 0.5f * sin_roll * sin_pitch * sin_yaw;
	jacob(3,1) = -0.5f * cos_roll * sin_pitch * sin_yaw - 0.5f * sin_roll * cos_pitch * cos_yaw;

	// Yaw derivatives
	jacob(0,2) = -0.5f * cos_roll * cos_pitch * sin_yaw + 0.5f * sin_roll * sin_pitch * cos_yaw;
	jacob(1,2) = -0.5f * sin_roll * cos_pitch * sin_yaw - 0.5f * cos_roll * sin_pitch * cos_yaw;
	jacob(2,2) = -0.5f * cos_roll * sin_pitch * sin_yaw + 0.5f * sin_roll * cos_pitch * cos_yaw;
	jacob(3,2) =  0.5f * cos_roll * cos_pitch * cos_yaw + 0.5f * sin_roll * sin_pitch * sin_yaw;

	return jacob;
}