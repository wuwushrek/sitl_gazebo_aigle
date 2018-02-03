#ifndef IO_AIGLE_SITL
#define IO_AIGLE_SITL

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <poll.h>
#include <unistd.h>

#include <mavlink/v2.0/common/mavlink.h>
// #include <gazebo/common/common.hh>

#include "aigle_types.h"

#define DEFAULT_AIGLE_UDP_PORT 14570
#define SOCKET_NAME "/tmp/share_fd_gazebo_aigle.socket"

class IoAigleInterface{

public:

	uint8_t succeed_init;
	bool new_data;
	
	IoAigleInterface();

	IoAigleInterface(int port_to_gazebo);

	void readMotors();
	void updateGPS(const gps_data* m_position);
	void writeMotors(const motor_data *motor_vals);
	const motor_data* get_motor_data();

	~IoAigleInterface();

private:

	const static uint32_t pwm_max_value =  2000;
	const static uint32_t pwm_min_value = 1000;
	const static unsigned int mode_flag_armed = 128; // following MAVLink spec
	const static unsigned int mode_flag_custom = 1;

	motor_data received_motors_values_;

	gps_data gps_pos_vel_;

	int port_to_gazebo_;

	int fd_motors_recv_;
	int fd_motors_send_;

	struct sockaddr_in myaddr_;

	struct sockaddr_in srcaddr_;
	socklen_t addrlen_;

	struct sockaddr_in destaddr_;
	socklen_t destaddrlen_;

	unsigned char buf_[1024];

	struct pollfd fds[1];

	
	void send_mavlink_message(const mavlink_message_t *message);
	int receive_fd();
	void init_io_interface(int port_to_gazebo);
};

#endif