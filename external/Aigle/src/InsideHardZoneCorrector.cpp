#include "OutOfZoneCorrector.h"
#include <stdio.h>

InsideHardZoneCorrector::InsideHardZoneCorrector(AigleIoInterface *aigleIo){
	this->aigleIo = aigleIo;
	correctorID = CorrectorType::e_HardZoneCorrector;
	for (uint8_t i =0 ; i< n_FaultType ; i++){
		correctedFaults[i] = false;
	}
	correctedFaults[FaultType::e_HardZoneFault] =  true;
}

InsideHardZoneCorrector::~InsideHardZoneCorrector(){
	printf("Inside Hard zone corrector deleted ! \n");
}

CorrectorType InsideHardZoneCorrector::getCorrectorType() const {
	return this->correctorID;
}

bool InsideHardZoneCorrector::isFaultCorrected(FaultType faultType) const{
	return correctedFaults[faultType];
}

void InsideHardZoneCorrector::correctFault(const EstimateState *estimate_state) const{
	this->aigleIo->stopMotors();
}