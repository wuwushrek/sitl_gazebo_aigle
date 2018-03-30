#include "NoMotorCommandFault.h"
#include <stdio.h>

NoMotorCommandFault::NoMotorCommandFault(uint64_t min_delay){
	this->min_delay = min_delay;
	this->faultID = FaultType::e_NoMotorCommandFault;
}

NoMotorCommandFault::~NoMotorCommandFault(){
	printf("No Motor Command Fault deleted ! \n");
}

FaultType NoMotorCommandFault::getFaultType() const {
	return this->faultID;
}

bool NoMotorCommandFault::detectFault(const EstimateState *estimate_state) const {
	return (xTaskGetTickCount() * portTICK_PERIOD_MS - estimate_state->last_time_motors_received)>min_delay;
}
