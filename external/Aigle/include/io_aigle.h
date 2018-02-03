#ifndef IO_AIGLE
#define IO_AIGLE

#include "aigle_types.h"

class IoAigleInterface{

public:
	IoAigleInterface();

	void readMotors();
	void updateGPS(const gps_data* m_position);
	void writeMotors(const motor_data *motor_vals);
	const motor_data* get_motor_data();

	~IoAigleInterface();

private:
	motor_data received_motors_values_;
};

#endif  //IO_AIGLE