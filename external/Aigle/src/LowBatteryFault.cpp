#include "LowBatteryFault.h"
#include <stdio.h>

LowBatteryFault::LowBatteryFault(uint8_t min_bat){
	this->min_battery_level = min_bat;
	this->faultID = FaultType::e_LowBatteryFault;
}

LowBatteryFault::~LowBatteryFault(){
	printf("Low Battery Fault deleted ! \n");
}

FaultType LowBatteryFault::getFaultType() const {
	return this->faultID;
}

bool LowBatteryFault::detectFault(const EstimateState *estimate_state) const {
	return estimate_state->battery_level < min_battery_level;
}