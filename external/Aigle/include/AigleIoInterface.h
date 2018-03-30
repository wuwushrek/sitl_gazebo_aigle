#ifndef __AIGLE_IO_INTERFACE_H__
#define __AIGLE_IO_INTERFACE_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "AigleTypes.h"

class AigleIoInterface {
	
protected:
	QueueHandle_t gpsQueueHandle_;
	QueueHandle_t imuQueueHandle_;
	QueueHandle_t ssQueueHandle_;

public:
	virtual void readIMU() = 0;
	virtual void readGPS() = 0;

	virtual void transferMotorsValue() = 0;
	virtual void writeMotors(const MotorData *motor_vals) = 0;
	virtual void stopMotors() = 0;

	// Queue for IMU and GPS data values
	virtual void setGPSQueue(QueueHandle_t gpsQueueHandle) = 0;
	virtual void setIMUQueue(QueueHandle_t imuQueueHandle) = 0;
	virtual void setSystemStatusQueue(QueueHandle_t ssQueueHandle) = 0;

	// Just for simulation
	virtual void collectSensorsData() = 0;
};

#endif //__AIGLE_IO_INTERFACE_H__