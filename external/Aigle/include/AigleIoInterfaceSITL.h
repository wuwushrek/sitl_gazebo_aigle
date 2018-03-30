#ifndef __IO_AIGLE_SITL_H__
#define __IO_AIGLE_SITL_H__

#include "AigleIoInterface.h"

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

#define ITERATION_GPS 100
#define ITERATION_IMU 10

#define DEFAULT_AIGLE_UDP_PORT 14570
#define SOCKET_NAME "/tmp/share_fd_gazebo_aigle.socket"

class AigleIoInterfaceSITL : public AigleIoInterface{

public:
	
	uint8_t succeed_init;
	
	AigleIoInterfaceSITL();
	AigleIoInterfaceSITL(int port_to_gazebo);

	virtual void readIMU();
	virtual void readGPS();

	virtual void transferMotorsValue();
	virtual void writeMotors(const MotorData *motor_vals);
	virtual void stopMotors();

	// Queue for IMU and GPS data values
	virtual void setGPSQueue(QueueHandle_t gpsQueueHandle);
	virtual void setIMUQueue(QueueHandle_t imuQueueHandle);
	virtual void setSystemStatusQueue(QueueHandle_t ssQueueHandle);

	// utility function just for simulation
	virtual void collectSensorsData();

	~AigleIoInterfaceSITL();

private:

	//const static uint32_t pwm_max_value =  2000;
	//const static uint32_t pwm_min_value = 1000;

	// following MAVLink spec data
	const static unsigned int mode_flag_armed = 128;
	const static unsigned int mode_flag_custom = 1;

	// motor value update flag
	bool motors_updated;
	uint64_t lastTimeMotors;

	// Simulated sensor values
	MotorData received_motors_values_;
	GpsData gps_pos_vel_;
	ImuData imu_raw_;

	// Queue for GPS and IMU outcome values
	QueueHandle_t gpsQueueHandle_;
	QueueHandle_t imuQueueHandle_;

	/* UDP communication structures attributes */
	int port_to_gazebo_;

	int fd_motors_recv_;
	int fd_motors_send_;

	struct sockaddr_in myaddr_;
	struct sockaddr_in srcaddr_;
	socklen_t addrlen_;

	unsigned char buf_[1024];

	// Polling structure for reception event on a socket
	struct pollfd fds[2];

	void send_mavlink_message(const mavlink_message_t *message);

	void checkReceivedData(uint8_t index, struct sockaddr_in *destaddr, socklen_t *destaddrlen);

	void init_io_interface(int port_to_gazebo);
	
	void initializeIoInterface(int port_to_gazebo);

	int do_useless_calculation(uint32_t seq);
};

#endif // __IO_AIGLE_SITL_H__