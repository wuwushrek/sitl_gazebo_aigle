#include "OutOfZoneFault.h"
#include "stdio.h"

OutOfZoneFault::OutOfZoneFault(Area *area, FaultType faultID){
	this->faultID = faultID;
	this->area = area;
}

OutOfZoneFault::~OutOfZoneFault(){
	printf("Out of Zone fault deleted ! \n");
}

bool OutOfZoneFault::detectFault(const EstimateState * estimate_state) const {
	return this->area->outOfArea(estimate_state);
}

FaultType OutOfZoneFault::getFaultType() const {
	return this->faultID;
}