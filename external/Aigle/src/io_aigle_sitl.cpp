#include "io_aigle_sitl.h"

/*
* Empty constructor for IoAigleInterface
*/
IoAigleInterface::IoAigleInterface(){}

/*
* IoAigleInterface constructor : port_to_gazebo contains the value of the port
* to use to send motor information to gazebo
*/
IoAigleInterface::IoAigleInterface(int port_to_gazebo){
	// Constructor doesn't failed to initalize
	succeed_init = 1;
	new_data = false;

	//Initialize io port and socket associated to all of them
	init_io_interface(port_to_gazebo);

	// Receive UDP file descriptor for motors command
	fd_motors_recv_ = receive_fd();
	if (fd_motors_recv_ < 0){
		std::cout << "[AIGLE] Failed to receive socket for motors data !" << std::endl;
		succeed_init = 0;
	} else {
		std::cout << "[AIGLE] PX4 socket = " << fd_motors_recv_ << std::endl;
	}

	//Initialize fds poll structure
	fds[0].events = POLLIN;
	fds[0].fd = fd_motors_recv_;

	// Initialize motors as disarmed
	received_motors_values_.is_armed = 0;
	for (unsigned int i =0; i< n_out_max ; i++){
		received_motors_values_.motor_values[i] = 0;
	}
}

IoAigleInterface::~IoAigleInterface(){
	std::cout << "[AIGLE] Destruction of IoAigleInterface reference" << std::endl;
}

/*
* Get the last saved motors values
*/
const motor_data* IoAigleInterface::get_motor_data(){
	return &received_motors_values_;
}

/*
* Read PWM or motors value from PX4 SITL
*/
void IoAigleInterface::readMotors(){
	::poll(&fds[0], (sizeof(fds[0]) /sizeof(fds[0])), 0);
	if (fds[0].revents & POLLIN){
		int len;
		if ( (len = recvfrom(fd_motors_recv_, buf_, sizeof(buf_), 0, (struct sockaddr *) &destaddr_, &destaddrlen_)) < 0){
			return ;
		}
		new_data = true;
		mavlink_message_t msg;
		mavlink_status_t status;
		for (unsigned int i = 0; i<len ; i++){
			if(mavlink_parse_char(MAVLINK_COMM_0, buf_[i], &msg, &status)){
				if (msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS){
					mavlink_hil_actuator_controls_t controls;
					mavlink_msg_hil_actuator_controls_decode(&msg , &controls);
					// std::cout << "MOTORS : ";
					received_motors_values_.is_armed = ((controls.mode  & MAV_MODE_FLAG_SAFETY_ARMED) > 0) ? 1 : 0;
					// std::cout << received_motors_values_.is_armed << " , ";
					for (unsigned int j =0; j<n_out_max ; j++){
						received_motors_values_.motor_values[j] = ((uint32_t) ((controls.controls[j] + 1.0)*(pwm_max_value - pwm_min_value)/2.0)) + pwm_min_value;
						// std::cout << received_motors_values_.motor_values[j] << " , ";
					}
					std::cout << std::endl;
				}
			}
		}
	}
}

/*
* Update data from GPS sensors 
*/
void IoAigleInterface::updateGPS(const gps_data* m_position){

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
	new_data = false;
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
int IoAigleInterface::receive_fd(){

	// Create a UNIX socket and the addresses associated with the socket
	int fd_unix;
	struct sockaddr_un send_addr;

	// Try to initalize the socket as TCP socket
	if((fd_unix = socket(AF_UNIX, SOCK_STREAM , 0)) < 0){
		std::cout << "[AIGLE] Failed to create Local UNIX socket !" << std::endl;
		succeed_init = 0;
		return -1;
	}

	// Initialize the current socket address 
	memset(&send_addr , 0 , sizeof(struct sockaddr_un));
	send_addr.sun_family =  AF_UNIX;
	strcpy(send_addr.sun_path , SOCKET_NAME);

	// Try to connect to the local server
	while ( connect(fd_unix , (struct sockaddr *) &send_addr , ( strlen(send_addr.sun_path) + sizeof(send_addr.sun_family))) < 0){
		std::cout << "[AIGLE] Failed to connect to PX4 SITL !" << std::endl;
	}

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
	while(recvmsg(fd_unix,&msg,0) < 0) {
		std::cout << "[AIGLE] Trying to get fd from robot plugin !!" << std::endl;
	}

	// Extract the ancillary data we wanted to obtain
	struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
	int result;
	memmove(&result, CMSG_DATA(cmsg), sizeof(result));

	// Close the socket since not use anymore
	close(fd_unix);

	return result;
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

	memset((char * )&myaddr_, 0 , sizeof(myaddr_));	// Initialize myaddr variable	

	myaddr_.sin_family = AF_INET;
	myaddr_.sin_addr.s_addr = htonl(INADDR_ANY);	// Let the OS pick the interface
	myaddr_.sin_port = htons(0);		// Let the OS pick the port

	// Bind the socket to the addr
	if (bind(fd_motors_send_, (struct sockaddr *)&myaddr_, sizeof(myaddr_)) < 0){
      std::cout << "[AIGLE] bind Aigle failed" << std::endl;
      succeed_init = 0;
      return;
    }

	//Initialize the destination address
	memset( (char * )&srcaddr_ , 0 , sizeof(srcaddr_));
	srcaddr_.sin_family = AF_INET;
	srcaddr_.sin_addr.s_addr =  htonl(INADDR_ANY);
	srcaddr_.sin_port = htons(port_to_gazebo_);
	addrlen_ = sizeof(srcaddr_);

}