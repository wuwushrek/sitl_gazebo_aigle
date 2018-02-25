#include "io_aigle_sitl.h"

int receive_fd();

/*
* Empty constructor for IoAigleInterface
*/
IoAigleInterface::IoAigleInterface(){
	motors_updated = false;
	initializeIoInterface(DEFAULT_AIGLE_UDP_PORT);
}

IoAigleInterface::IoAigleInterface(int port_to_gazebo){
	motors_updated = false;
	initializeIoInterface(port_to_gazebo);
}

IoAigleInterface::~IoAigleInterface(){
	close(fd_motors_recv_);
	close(fd_motors_send_);
	std::cout << "[AIGLE] Destruction of IoAigleInterface reference" << std::endl;
}

// std::cout << "[AIGLE_GPS] lat = " << m_position->lat << " ; lon= " << m_position->lon << " ; alt= " << m_position->alt << std::endl;
// std::cout << "[AIGLE_GPS] vn = " << m_position->vn << " ; ve= " << m_position->ve << " ; vd= " << m_position->vd << std::endl;

// std::cout << "[AIGLE] xacc = " << m_imu->xacc << " ; yacc= " << m_imu->yacc << " ; zacc= " << m_imu->zacc << std::endl;
// std::cout << "[AIGLE] xgyro = " << m_imu->xgyro << " ; ygyro= " << m_imu->ygyro << " ; zgyro= " << m_imu->zgyro << std::endl;

/*
* Update data from GPS sensors 
*/
void IoAigleInterface::readGPS(){
	do_useless_calculation(ITERATION_GPS);
}

/*
* Update data from IMU sensors 
*/
void IoAigleInterface::readIMU(){
	do_useless_calculation(ITERATION_IMU);
}

/*
* Output PWM motors values
*/
void IoAigleInterface::writeMotors(const motor_data *motors_values){
	mavlink_hil_actuator_controls_t controls = {};
	mavlink_message_t message = {};
	for (unsigned int i =0; i<n_out_max ; i++){
		controls.controls[i] = ((motors_values->motor_values[i] - pwm_min_value)*2.0/(pwm_max_value- pwm_min_value)) - 1.0;
	}
	controls.mode = mode_flag_custom;
	controls.mode |= (motors_values->is_armed == 1) ? mode_flag_armed : 0;
	controls.flags = 0;

	mavlink_msg_hil_actuator_controls_encode(0,200,&message,&controls);
	send_mavlink_message(&message);
	motors_updated = false;
}

/*
* Simple function to directly retransmit current motors output
*/
void IoAigleInterface::transferMotorsValue(){
	if ( ! motors_updated){
		return ;
	}
	// Copy in order to avoid synchronization issue
	motor_data send_motor = received_motors_values_;

	// Write to the gazebo plugin
	writeMotors(&send_motor);
}

/*
* Collect in a thread safe way gps , IMU and motors values via various udp socket
*/
void IoAigleInterface::collectSensorsData(){
	// Collect data from PX4 (motors output)
	checkReceivedData(0, NULL , NULL);
	// Collect data from aigle plugin in gazebo (simulated sensors)
	checkReceivedData(1, &srcaddr_ , &addrlen_);
}

/* Initialize fd for motors output and fd for sensours input from Simulator */
void IoAigleInterface::initializeIoInterface(int port_to_gazebo){
	// Constructor doesn't failed to initalize
	succeed_init = 1;

	//Initialize io port and socket associated to all of them
	init_io_interface(port_to_gazebo);

	// Receive UDP file descriptor for motors command
	std::cout << "Trying to received fd from for motors data..." << std::endl;
	fd_motors_recv_ = receive_fd();
	if (fd_motors_recv_ < 0){
		std::cout << "[AIGLE] Failed to receive socket for motors data !" << std::endl;
		succeed_init = 0;
		return ;
	}
	//Initialize fds poll structure
	fds[0].events = POLLIN;
	fds[0].fd = fd_motors_recv_;
	fds[1].events = POLLIN;
	fds[1].fd = fd_motors_send_;

	// Initialize motors as disarmed
	received_motors_values_.is_armed = 0;
	for (unsigned int i =0; i< n_out_max ; i++){
		received_motors_values_.motor_values[i] = pwm_min_value;
	}
}

/*
*	This function helps with the initialization of the gazebo port and the 
* 	sockets associated with that port + binding + definition of the address of
*	destination
*/
void IoAigleInterface::init_io_interface(int port_to_gazebo){

	//Copy the port 
	port_to_gazebo_ = port_to_gazebo;

	//Initialize the socket
	if ((fd_motors_send_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
		std::cout << "[AIGLE] Create socket failed ! " << std::endl;
		succeed_init = 0;
		return;
	}

	memset((char *)&myaddr_, 0, sizeof(myaddr_));
	myaddr_.sin_family = AF_INET;
	myaddr_.sin_addr.s_addr = htonl(INADDR_ANY);
	// Let the OS pick the port
	myaddr_.sin_port = htons(port_to_gazebo_);

	//Bind socket with the file descriptor and address
	if (bind(fd_motors_send_, (struct sockaddr *)&myaddr_, sizeof(myaddr_)) < 0) {
		std::cout << "[AIGLE] Failed to bind socket : fd_motors_send_" << std::endl;
		succeed_init = 0;
		return;
	}

	//Initialize the destination address for motor output
	memset( (char * )&srcaddr_ , 0 , sizeof(srcaddr_));
	srcaddr_.sin_family = AF_INET;
	srcaddr_.sin_addr.s_addr =  htonl(INADDR_ANY);
	srcaddr_.sin_port = htons(port_to_gazebo_);
	addrlen_ = sizeof(srcaddr_);

	std::cout << "[AIGLE] socket fd_motors_send_ created ! " << std::endl;
}

int IoAigleInterface::do_useless_calculation(uint32_t max_count){
	int res = 0;
	for (uint32_t i = 0; i< max_count ; i++){
		for(uint32_t j =0; j< max_count ; j++){
			res += j-i;
		}
	}
	return res;
}

/*
* Routine to send a mavlink message over UDP
*/
void IoAigleInterface::send_mavlink_message(const mavlink_message_t *message){
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int packetlen = mavlink_msg_to_send_buffer(buffer, message);
  ssize_t len = sendto(fd_motors_send_, buffer, packetlen, 0, (struct sockaddr *) &srcaddr_, sizeof(srcaddr_));
  if (len <= 0) {
    std::cout << "[AIGLE] Failed sending mavlink message\n";
  }
}

/*
* Another Routine to receive the file descriptor from another process
* Here the file descriptor will point at the mavlink port to receive motors data
*/
int receive_fd(){

	// Create a UNIX socket and the addresses associated with the socket
	int fd_unix;
	struct sockaddr_un send_addr;

	// Try to initalize the socket as TCP socket
	if((fd_unix = socket(AF_UNIX, SOCK_STREAM , 0)) < 0){
		std::cout << "[AIGLE] Failed to create Local UNIX socket !" << std::endl;
		return -1;
	}

	// Initialize the current socket address 
	memset(&send_addr , 0 , sizeof(struct sockaddr_un));
	send_addr.sun_family =  AF_UNIX;
	strcpy(send_addr.sun_path , SOCKET_NAME);

	// Try to connect to the local server
	while ( connect(fd_unix , (struct sockaddr *) &send_addr , ( strlen(send_addr.sun_path) + sizeof(send_addr.sun_family))) < 0);

	// Define structure for sending file descriptor amongst different process
	struct msghdr msg = {0};
	char useless[4];

	//Structure buffer for sending message with ancillary datas
	struct iovec io[1];
	io[0].iov_base = (void *) useless;
	io[0].iov_len = sizeof(useless);

	// Fill the message structure with the message we want to send
	msg.msg_iov = io;
	msg.msg_iovlen = 1;

	// Initialize the anicllary data structure
	char buf[256];
	msg.msg_control = buf;
	msg.msg_controllen = sizeof(buf);

	// Try to receive the message with the file descriptor information
	if (recvmsg(fd_unix,&msg,0) <= 0) {
		std::cout << "[AIGLE] Failed during recption !!" << std::endl;
		return -1;
	}

	// Extract the ancillary data we wanted to obtain
	struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
	int result;
	memmove(&result, CMSG_DATA(cmsg), sizeof(result));
	std::cout << "[AIGLE] PX4 socket fd received = " << result <<  std::endl;

	// Close the socket since not used anymore
	close(fd_unix);

	return result;
}

void IoAigleInterface::checkReceivedData(uint8_t index, struct sockaddr_in *destaddr, socklen_t *destaddrlen){
	::poll(&fds[index], (sizeof(fds[index]) /sizeof(fds[index])), 0);
	if (fds[index].revents & POLLIN){
		int len;
		if ( (len = recvfrom(fds[index].fd , buf_, sizeof(buf_), 0, (struct sockaddr *) destaddr, destaddrlen)) < 0){
			return ;
		}
		// std::cout << (int) len << std::endl;
		mavlink_message_t msg;
		mavlink_status_t status;
		for (unsigned int i = 0; i<len ; i++){
			if(mavlink_parse_char(MAVLINK_COMM_0, buf_[i], &msg, &status)){
				if (msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS){
					mavlink_hil_actuator_controls_t controls;
					mavlink_msg_hil_actuator_controls_decode(&msg , &controls);
					// std::cout << "MOTORS : " ;
					received_motors_values_.is_armed = ((controls.mode  & MAV_MODE_FLAG_SAFETY_ARMED) > 0) ? 1 : 0;
					// std::cout << received_motors_values_.is_armed << " , ";
					for (unsigned int j =0; j<n_out_max ; j++){
						received_motors_values_.motor_values[j] = ((uint32_t) ((controls.controls[j] + 1.0)*(pwm_max_value - pwm_min_value)/2.0)) + pwm_min_value;
						// std::cout << received_motors_values_.motor_values[j] << " , ";
					}
					motors_updated = true;
					// std::cout << std::endl;
				} else if (msg.msgid == MAVLINK_MSG_ID_HIL_SENSOR) {
					mavlink_hil_sensor_t imu;
					mavlink_msg_hil_sensor_decode(&msg, &imu);
					// std::cout << "IMU xacc = " << imu.xacc << std::endl;
					imu_raw_.time_usec = imu.time_usec;
					imu_raw_.xacc = imu.xacc;
					imu_raw_.yacc = imu.yacc;
					imu_raw_.zacc = imu.zacc;
					imu_raw_.xgyro = imu.xgyro;
					imu_raw_.ygyro = imu.ygyro;
					imu_raw_.zgyro = imu.zgyro;
					imu_raw_.xmag = imu.xmag;
					imu_raw_.ymag = imu.ymag;
					imu_raw_.zmag = imu.zmag;
					imu_raw_.abs_pressure = imu.abs_pressure;
					imu_raw_.diff_pressure = imu.diff_pressure;
					imu_raw_.pressure_alt = imu.pressure_alt;
					imu_raw_.temperature = imu.temperature;
					imu_raw_.fields_updated = imu.fields_updated;
				} else if (msg.msgid == MAVLINK_MSG_ID_HIL_GPS){
					mavlink_hil_gps_t gps;
					mavlink_msg_hil_gps_decode(&msg, &gps);
					// std::cout << "GPS lat = " << gps.lat << std::endl;
					gps_pos_vel_.time_usec = gps.time_usec;
					gps_pos_vel_.lat = gps.lat;
					gps_pos_vel_.lon = gps.lon;
					gps_pos_vel_.alt = gps.alt;
					gps_pos_vel_.eph = gps.eph;
					gps_pos_vel_.epv = gps.epv;
					gps_pos_vel_.vel = gps.vel;
					gps_pos_vel_.vn = gps.vn;
					gps_pos_vel_.ve = gps.ve;
					gps_pos_vel_.vd = gps.vd;
					gps_pos_vel_.cog = gps_pos_vel_.cog;
					gps_pos_vel_.fix_type = gps.fix_type;
					gps_pos_vel_.satellites_visible = gps.satellites_visible;
				}
			}
		}
	}
}