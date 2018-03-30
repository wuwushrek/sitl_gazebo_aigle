#include "NoMotorLowBatteryCorrector.h"
#include <stdio.h>

NoMotorLowBatteryCorrector::NoMotorLowBatteryCorrector(AigleIoInterface *aigleIo){
	this->aigleIo = aigleIo;
	correctorID = CorrectorType::e_NoMotorLowBatteryCorrector;
	for (uint8_t i =0 ; i< n_FaultType ; i++){
		correctedFaults[i] = false;
	}
	correctedFaults[FaultType::e_NoMotorCommandFault] =  true;
	correctedFaults[FaultType::e_LowBatteryFault] =  true;
}

NoMotorLowBatteryCorrector::~NoMotorLowBatteryCorrector(){
	printf("No Motor and Low Battery corrector deleted ! \n");
}

CorrectorType NoMotorLowBatteryCorrector::getCorrectorType() const {
	return this->correctorID;
}

bool NoMotorLowBatteryCorrector::isFaultCorrected(FaultType faultType) const{
	return correctedFaults[faultType];
}

void NoMotorLowBatteryCorrector::correctFault(const EstimateState *estimate_state) const{
	this->aigleIo->stopMotors();
}