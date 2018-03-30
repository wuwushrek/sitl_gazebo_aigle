
#include "Estimator.h"

using namespace matrix;

Matrix3f skew_symmetric(const matrix::Vector3f &v){
	Matrix3f m;
	// Diag are zero elements
	m(0,0) = 0;
	m(1,1) = 0;
	m(2,2) = 0;
	// Other elements are symmetric opposite
	m(0,1) = - v(2);
	m(1,0) =   v(2);

	m(0,2) =   v(1);
	m(2,0) = - v(1);

	m(1,2) = - v(0);
	m(2,1) =   v(0);
	return m;
}

//diff_(p*q) /diff_q
Matrix4f diff_pq_q(const matrix::Quatf &p){
	uint8_t i,j;
	Matrix4f res;

	res(0,0) =  p(0);
	Vector3f v(p.data()+ 1);
	for(i=1 ; i< 4 ; i++ ){
		res(0,i) =	-p(i);
		res(i,0) =	p(i);
	}
	Matrix3f v_skew = skew_symmetric(v);
	for(i= 0 ; i< 3 ; i++){
		for(j = 0 ; j <3 ; j++){
			res(i+1,j+1) = v_skew(i,j);
		}
		res(i+1,i+1) += p(0);
	}
	return res; 
}

//diff_(p*q)/ diff_p
Matrix4f diff_pq_p(const matrix::Quatf &q){
	uint8_t i,j;
	Matrix4f res;

	res(0,0) =  q(0);
	Vector3f v(q.data()+ 1);
	for(i=1 ; i< 4 ; i++ ){
		res(0,i) =	-q(i);
		res(i,0) =	q(i);
	}
	Matrix3f v_skew = skew_symmetric(v);
	for(i= 0 ; i< 3 ; i++){
		for(j = 0 ; j <3 ; j++){
			res(i+1,j+1) = -v_skew(i,j);
		}
		res(i+1,i+1) += q(0);
	}
	return res; 
}

//diff_(q*v*q_star)/ diff_q
Matrix34f diff_qvqstar_q(const matrix::Quatf &q, const matrix::Vector3f &v){
	uint8_t i,j;
	Matrix34f res;

	Vector3f qv(q.data()+ 1);

	Matrix3f qv_skew = skew_symmetric(qv);
	Matrix3f v_skew = skew_symmetric(v);

	Vector3f qv_skew_v = qv_skew * v;
	for (i=0 ; i<3 ; i++){
		res(i,0) = 2 * (q(0)* v(i) + qv_skew_v(i)); 
	}

	Matrix3f vqstar_qvstar = - ((Matrix<float , 3 , 1>) v) *qv.transpose() + ((Matrix<float , 3 , 1>) qv) * v.transpose();
	float vdotq_2 = 2 * (v(0)*qv(0) + v(1)*qv(1) + v(2)*qv(2));
	for(i = 0 ; i< 3 ; i++){
		for (j=0 ; j<3 ; j++){
			res(i , j+1) = 2 * (vqstar_qvstar(i,j) - q(0) * v_skew(i,j));
		}
		res(i, i+1) += vdotq_2; 
	}
	return res;
}

//diff_(qstar*v*q)/ diff_q
Matrix34f diff_qstarvq_q(const matrix::Quatf &q, const matrix::Vector3f &v){
	uint8_t i,j;
	Matrix34f res;

	Vector3f qv(q.data()+ 1);

	Matrix3f qv_skew = skew_symmetric(qv);
	Matrix3f v_skew = skew_symmetric(v);

	Vector3f qv_skew_v = qv_skew * v;
	for (i=0 ; i<3 ; i++){
		res(i,0) = 2 * (q(0)* v(i) - qv_skew_v(i)); 
	}

	Matrix3f vqstar_qvstar = - ((Matrix<float , 3 , 1>) v) *qv.transpose() + ((Matrix<float , 3 , 1>) qv) * v.transpose();
	float vdotq_2 = 2 * (v(0)*qv(0) + v(1)*qv(1) + v(2)*qv(2));
	for(i = 0 ; i< 3 ; i++){
		for (j=0 ; j<3 ; j++){
			res(i , j+1) = 2 * (vqstar_qvstar(i,j) + q(0) * v_skew(i,j));
		}
		res(i, i+1) += vdotq_2; 
	}
	return res;
}

//diff_(q*v*q_star)/ diff_v
Matrix3f diff_qvqstar_v(const matrix::Quatf &q){
	uint8_t i,j;
	Matrix3f res;

	Vector3f qv(q.data()+ 1);
	Matrix3f qv_skew = skew_symmetric(qv);
	Matrix3f temp  =  ((Matrix<float , 3 , 1>) qv) * qv.transpose();
	float q0_qdotq = q(0) * q(0) - (qv(0) * qv(0) + qv(1) * qv(1) + qv(2)* qv(2));

	for (i=0; i< 3 ; i++){
		for (j=0; j<3 ; j++){
			res(i,j) = 2 * ( temp(i,j) + q(0) * qv_skew(i,j));
		}
		res(i, i) += q0_qdotq; 
	}
	return res;
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

	identity.setIdentity();
	
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

void Estimator::setLastTimeMotorReceived(uint64_t *last_time){
	last_time_motors_received = last_time;
}

void Estimator::setGPSQueue(QueueHandle_t gpsQueueHandle){
	_gpsQueueHandle = gpsQueueHandle;
}

void Estimator::setIMUQueue(QueueHandle_t imuQueueHandle){
	_imuQueueHandle = imuQueueHandle;
}

void Estimator::setEstimateStateQueue(QueueHandle_t estimQueue){
	_estimQueue = estimQueue;
}

void Estimator::setSystemStatusQueue(QueueHandle_t ssQueue){
	_ssQueue = ssQueue;
}

void Estimator::collectInitSample (const ImuData &last_imu , float dt){
	// We calculate an riemann sum while max_init_sample isn't reached
	init_g_b += Vector3f(last_imu.xacc * dt , last_imu.yacc * dt , last_imu.zacc * dt) ;  
	init_mag_b += Vector3f(last_imu.xmag * dt , last_imu.ymag * dt , last_imu.zmag * dt);
	init_gyro_bias_b += Vector3f(last_imu.xgyro * dt , last_imu.ygyro * dt , last_imu.zgyro * dt);

	// sum of all gps alt lat and lon
	GpsData gps;
	if ( xQueueReceive(_gpsQueueHandle , &gps , ( TickType_t ) 0 ) ){
		gpsDataCount+=1;
		initgpsalt += (gps.alt * 1.0e-3);
		initgpslat += (gps.lat * 1.0e-7);
		initgpslon += (gps.lon * 1.0e-7);
		std::cout << "GPS received " << std::endl;
	} 

	// Increment number of imu data taken
	_sample_obtained+=1;
}

void Estimator::update(){
		ImuData last_imu;

	// Check if there is any updated IMU data with 50ms as max blocking time
	if ( ! xQueueReceive(_imuQueueHandle , &last_imu , pdMS_TO_TICKS(0))){
		return ;
	}

	// get current time and deltat
	uint64_t currentTimeStamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
	float dt = (currentTimeStamp - _timeStamp)/1.0e3f;
	//_mag_timeout += (currentTimeStamp - _timeStamp);
	_timeStamp = currentTimeStamp;

	// Fill the input u with values from IMU sensor
	_u(U_ax) = last_imu.xacc;
	_u(U_ay) = last_imu.yacc;
	_u(U_az) = last_imu.zacc;
	_u(U_wx) = last_imu.xgyro;
	_u(U_wy) = last_imu.ygyro;
	_u(U_wz) = last_imu.zgyro;

	// Still gathering data for initial estimation
	if (_sample_obtained < max_init_sample){
		// Collect initial sample data
		collectInitSample(last_imu ,  dt);
		// Check if a sufficient number of data has been collected
		if (_sample_obtained >= max_init_sample){
			float time_sec = currentTimeStamp / 1.0e3f;
			initSystem(time_sec);
		}
		return ;
	}

	GpsData gps;
	bool send_estimate = false;
	if ( xQueueReceive(_gpsQueueHandle , &gps , ( TickType_t ) 0 ) ){
		float x , y;
		float z = (gps.alt * 1e-3) - initgpsalt;
		// project(m_pos->lat * 1e-7 ,m_pos->lon * 1e-7, &x , &y);
		project(gps.lat * 1e-7 ,gps.lon * 1e-7, &y , &x); // Because ENU coordinate system
		last_estimate.x = x;
		last_estimate.y = y;
		last_estimate.z = z;
		float yaw_angle = (gps.cog * 1.0e-2f);
		last_estimate.yaw = (yaw_angle>180 ? yaw_angle-360 : yaw_angle)*M_DEG_TO_RAD ;
		send_estimate = true;
	}
	SystemStatus ss;
	if (xQueueReceive(_ssQueue , &ss , ( TickType_t ) 0 ) ){
		last_estimate.battery_level = ss.battery_level;
		last_estimate.last_time_motors_received = ss.last_time_motors;
		send_estimate = true;
	}

	if (send_estimate)
		xQueueOverwrite(_estimQueue, (void * ) &last_estimate);
}

/*void Estimator::update(){
	// Updated values from IMU sensor
	ImuData last_imu;

	// Check if there is any updated IMU data with 50ms as max blocking time
	if ( ! xQueueReceive(_imuQueueHandle , &last_imu , pdMS_TO_TICKS(50))){
		return ;
	}

	// get current time and deltat
	uint64_t currentTimeStamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
	float dt = (currentTimeStamp - _timeStamp)/1.0e3f;
	_mag_timeout += (currentTimeStamp - _timeStamp);
	_timeStamp = currentTimeStamp;

	// Fill the input u with values from IMU sensor
	_u(U_ax) = last_imu.xacc;
	_u(U_ay) = last_imu.yacc;
	_u(U_az) = last_imu.zacc;
	_u(U_wx) = last_imu.xgyro;
	_u(U_wy) = last_imu.ygyro;
	_u(U_wz) = last_imu.zgyro;

	// Still gathering data for initial estimation
	if (_sample_obtained < max_init_sample){
		// Collect initial sample data
		collectInitSample(last_imu ,  dt);
		// Check if a sufficient number of data has been collected
		if (_sample_obtained >= max_init_sample){
			float time_sec = currentTimeStamp / 1.0e3f;
			initSystem(time_sec);
		}
		return ;
	}

	predict(dt);

	// if (_mag_timeout > mag_update_time) {
	// 	Vector3f acc_mes(last_imu.xacc , last_imu.yacc , last_imu.zacc);
	// 	imuCorrect(acc_mes);
	// }
	
	if(_mag_timeout > mag_update_time){
		Vector3f m_mag (last_imu.xmag ,  last_imu.ymag ,  last_imu.zmag);
		yawCorrect(m_mag);
		_mag_timeout = 0;
	}

	GpsData gps;
	if ( xQueueReceive(_gpsQueueHandle , &gps , ( TickType_t ) 0 ) ){
		gpsCorrect(&gps);
	}
	// std::cout << "Delay = " << dt << std::endl;

}*/

void Estimator::initSystem(float currentTimeStamp){
	// Normalize data collected
	init_g_b /= currentTimeStamp ;
	init_mag_b /= currentTimeStamp;
	init_gyro_bias_b /= currentTimeStamp;

	// Set lat and lon in radian
	initgpslon /= (gpsDataCount); /// M_DEG_TO_RAD);
	initgpslat /= (gpsDataCount); /// M_DEG_TO_RAD);
	initgpslon *= M_DEG_TO_RAD;
	initgpslat *= M_DEG_TO_RAD;
	initgpsalt /= gpsDataCount ;
	cos_initgpslat = cos( initgpslat );
	sin_initgpslat = sin( initgpslat );

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
	_x(X_bax) = init_b_a(0);
	_x(X_bay) = init_b_a(1);
	_x(X_baz) = init_b_a(2);

	// _Rbn = Dcmf(Quatf(_x.data() + X_qw));
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
		_P(i,i) = 0.00005;
	}
	// set covariance matrix for iniial attitude 
	// for (i = 0 ; i< 4 ; i++){
	// 	for (j = 0 ; j< 4 ; j++){
	// 		_P(X_qw+i,X_qw+j) = cov_Q(i,j);
	// 	}
	// }
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

	// Init last estimate
	last_estimate.x = 0;
	last_estimate.y = 0;
	last_estimate.z = 0;
	last_estimate.yaw = yaw_init;
	SystemStatus ss;
	if (xQueueReceive(_ssQueue , &ss , ( TickType_t ) 100 ) ){
		last_estimate.battery_level = ss.battery_level;
		last_estimate.last_time_motors_received = ss.last_time_motors;
	}
}

void Estimator::process(matrix::Vector<float , n_X> &xdot){
	uint8_t i , j;
	xdot.setZero();
	_Fx.setZero();
	_Fu.setZero();

	Quatf q(_x.data() + X_qw);

	Quatf gyro_q(0,0,0,0);
	gyro_q(1) = _u(U_wx) - _x(U_bwx);
	gyro_q(2) = _u(U_wy) - _x(U_bwy);
	gyro_q(3) = _u(U_wz) - _x(U_bwz);

	Quatf q_dot = q * gyro_q;
	q_dot /= 2;
	// quaternion derivation
	for ( i = 0 ; i< 4 ; i++){
		xdot(X_qw + i) = q_dot(i);
	}
	// Accelerometer and gyro biais are exponential
	for (i=0 ; i<3 ; i++){
		xdot(X_bwx + i) = - corr_time_gyro * _x(X_bwx + i);
		xdot(X_bax + i) = - corr_time_gyro * _x(X_bax + i);
	}
	// Position derivative
	for(i=0 ; i<3 ; i++){
		xdot(X_x + i) = _x(X_vx + i);
	}
	// Velocity derivative
	Quatf acc_b_q(0,0,0,0);
	acc_b_q(1) = _u(U_ax) - _x(U_bax);
	acc_b_q(2) = _u(U_ay) - _x(U_bay);
	acc_b_q(3) = _u(U_az) - _x(U_baz);
	Quatf acc_n_q = q * acc_b_q * q.inversed();
	for (i=0 ; i < 3 ; i++){
		xdot(X_vx + i) = acc_n_q(i+1) + gravity(i);
	}

	// Update jacobian in consideration of x vector 
	// Partial Derivative in respect of quaternion q
	Matrix4f temp  = diff_pq_p(gyro_q) * 0.5f;
	for(i=0 ; i< 4 ; i++){
		for (j=0 ; j< 4 ; j++){
			_Fx(X_qw + i , X_qw + j) = temp(i,j);
		}
	}
	temp = diff_pq_q(q) * (-0.5f);
	for (i=0 ; i< 4 ; i++){
		for (j=0 ; j< 3 ; j++){
			_Fx(X_qw + i , X_bwx + j) = temp(i, j+1);
		}
	}
	// Partial Derivative in respect of the accelerometer and gyro biais
	for(i=0 ; i< 3 ; i++){
		_Fx(X_bwx+i , X_bwx+i) = - corr_time_gyro;
		_Fx(X_bax+i , X_bax+i) = - corr_time_acc;
	}
	// Partial derivative in respect of the position
	for(i=0 ; i< 3 ; i++){
		_Fx(X_x + i , X_vx + i ) = 1;
	}
	// Partial derivative in respect of the velocity
	Matrix34f temp1 =  diff_qvqstar_q(q , acc_b_q.imag());
	for(i=0 ; i < 3 ; i++){
		for (j=0 ; j<4 ; j++){
			_Fx(X_vx + i , X_qw + j) = temp1(i,j);
		}
	}
	Matrix3f temp2 = - diff_qvqstar_v(q);
	for (i=0 ; i<3 ; i++){
		for (j=0 ; j<3 ; j++){
			_Fx(X_vx + i , X_bwx + j) = temp2(i,j);
		}
	}

	//Update F_u matrix
	for(i=0 ; i < 4 ; i++){
		for (j=0 ; j< 3 ; j++){
			_Fu(X_qw + i , U_wx+ j) = -_Fx(X_qw + i , X_bwx + j);
		}
	}
	for(i=0 ; i<3 ; i++){
		for(j=0 ; j<3 ; j++){
			_Fu(X_vx + i , U_ax + j) = - _Fx(X_vx + i , X_bwx + j);
		}
	}
	for(i=0 ; i< 3 ; i++){
		_Fu(X_bwx+ i , U_bwx+ i) = 1;
		_Fu(X_bax+ i , U_bax+ i) = 1;
	}
}

void Estimator::predict(float dt){
	uint8_t i;
	Vector<float, n_X> xdot;
	process(xdot);

	_x += xdot * dt;

	// Make quaternion unit
	Quatf q(_x.data() + X_qw);
	q  = q.unit();
	for(i=0 ; i< 4 ; i++){
		_x(X_qw + i ) = q(i);
	}

	// Update _Fx :  _Fx =  Identity + F*dt
	_Fx *= dt;
	for (i=0 ; i< n_X ; i++){
		_Fx(i , i) += 1;
	}
	// update _Fu : _Fu = _Fu * dt
	_Fu *= dt;

	// Set Process noise matrix Q
	for (i =0 ; i< 3 ; i++){
		_Q(U_bwx + i ,  U_bwx + i ) = sigma_bw * sigma_bw / dt;
		_Q(U_bax + i ,  U_bax + i ) = sigma_ba * sigma_ba / dt;
	}
	//Update error covariance matrix
	_P = _Fx * _P * _Fx.transpose() + _Fu * _Q * _Fu.transpose();

	for(i=0 ; i < n_X ; i++ ){
		if (_P(i,i) < 0)
			std::cout << " i = " << (int) i << std::endl;
	}

}


void Estimator::gpsCorrect(const GpsData *m_pos){
	/* Update position from position measurement */
	float x , y , vx , vy , vz;
	float z = (m_pos->alt * 1e-3) - initgpsalt;

	// project(m_pos->lat * 1e-7 ,m_pos->lon * 1e-7, &x , &y);
	project(m_pos->lat * 1e-7 ,m_pos->lon * 1e-7, &y , &x); // Because ENU coordinate system
	// vx = m_pos->ve * 1e-2f;
	// vy = m_pos->vn * 1e-2f;
	// vz = - m_pos->vd * 1e-2f;

	// h jacobian matrix

	EstimateState estimPos;
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
	//jacob_h(3, X_vx) = 1.0;
	//jacob_h(4, X_vy) = 1.0;
	//jacob_h(5, X_vz) = 1.0;
	Matrix<float , n_X , 3> jacob_h_transpose = jacob_h.transpose();

	// R measurement noise error
	SquareMatrix<float,3> R;
	R.setZero();
	R(0,0) = sigma_gps_x * sigma_gps_x;
	R(1,1) = sigma_gps_y * sigma_gps_y;
	R(2,2) = sigma_gps_z * sigma_gps_z;
	R(3,3) = sigma_gps_vx * sigma_gps_vx;
	R(4,4) = sigma_gps_vy * sigma_gps_vy;
	R(5,5) = sigma_gps_vz * sigma_gps_vz;

	SquareMatrix<float, 3> innov_cov = jacob_h * _P * jacob_h_transpose + R;
	Matrix<float , n_X , 3> kalman_gain = _P * jacob_h_transpose * inv(innov_cov);

	//float innov_data[6] = {x - _x(X_x) , y - _x(X_y) , z - _x(X_z),  vx - _x(X_vx), vy - _x(X_vy) , vz - _x(X_vz)};
	float innov_data[6] = {x - _x(X_x) , y - _x(X_y) , z - _x(X_z)};
	Vector<float,3> innov(innov_data);

	_x += kalman_gain * innov;
	_P = (identity - kalman_gain*jacob_h) * _P;
	// _P = _P * 0.5f  + _P.transpose()*0.5f;

	Quatf q(_x.data() + X_qw);
	q  = q.unit();
	for(uint8_t i=0 ; i< 4 ; i++){
		_x(X_qw + i ) = q(i);
	}

	//std:: cout << vx << " , " << vy << " , " << vz << std::endl;
	// std::cout << _x << std::endl;
	std::cout << _x(X_x) << " , " << _x(X_y) << " , " << _x(X_z)  << std::endl;
	std::cout << _x(X_vx) << " , " << _x(X_vy) << " , " << _x(X_vz)  << std::endl;
	// std::cout << "Diag : ";
	// for (uint8_t i=0 ; i< n_X ; i++){
	// 	std::cout << _P(i,i) << " ; ";
	// }
	// std::cout << std::endl;
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

	Quatf q(_x.data() + X_qw);
	Vector3f mag_data_n = q.conjugate(mag_data) ;
	mag_data_n(2) = 0.0;
	mag_data_n =  q.conjugate_inversed(mag_data_n);

	float yaw_mag = atan2(-mag_data_n(1) , mag_data_n(0));
	float yaw_est = atan2(num_atan, denom_atan);
	float yaw_innov =  yaw_mag - yaw_est;
	float innov_cov_yaw =  (jacob_h_yaw * _P * jacob_h_yaw.transpose())(0,0) + sigma_mag * sigma_mag;
	Matrix<float , n_X , 1> kalman_gain_yaw = (_P * jacob_h_yaw.transpose()) / innov_cov_yaw;
	_x += kalman_gain_yaw * yaw_innov;

	q =  Quatf(_x.data() + X_qw);
	q  = q.unit();
	for(uint8_t i=0 ; i< 4 ; i++){
		_x(X_qw + i ) = q(i);
	}

	_P = (identity - kalman_gain_yaw*jacob_h_yaw) * _P;
}

void Estimator::imuCorrect(const matrix::Vector3f &acc_data){
	Quatf q(_x.data() + X_qw);
	Quatf g_n_q(0 , 0 , 0 ,-1);
	Quatf acc_q = q.inversed() * g_n_q * q;

	Vector3f acc(acc_q.data() + 1);
	Matrix34f jacob_imu_temp = diff_qstarvq_q(q , gravity);
	Matrix<float , 3 , n_X> jacob_imu;
	jacob_imu.setZero();
	for (uint8_t i=0 ; i< 3 ; i++){
		for(uint8_t j=0 ; j<4 ; j++){
			jacob_imu(i , X_qw+j) = jacob_imu_temp(i,j);
		}
	}
	Vector3f acc_data_unit = acc_data.unit();

	SquareMatrix<float , 3> R;
	R.setZero();
	R(0,0) = sigma_gravity * sigma_gravity;
	R(1,1) = sigma_gravity * sigma_gravity;
	R(2,2) = sigma_gravity * sigma_gravity;

	SquareMatrix<float, 3> innov_cov = jacob_imu * _P * jacob_imu.transpose() + R;
	Matrix<float,n_X,3> kalman_gain = _P * jacob_imu.transpose() * inv(innov_cov);

	_x += kalman_gain * (acc_data_unit - acc);
	_P = (identity - kalman_gain * jacob_imu) * _P;

	q =  Quatf(_x.data() + X_qw);
	q  = q.unit();
	for(uint8_t i=0 ; i< 4 ; i++){
		_x(X_qw + i ) = q(i);
	}
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
