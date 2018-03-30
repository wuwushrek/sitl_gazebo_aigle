#include "StrategyDecisionMaker.h"
#include <stdio.h>

StrategyDecisionMaker::StrategyDecisionMaker(){}

StrategyDecisionMaker::~StrategyDecisionMaker(){
	printf(" Strategy Decision Maker deleted ! \n");
}

CorrectorType StrategyDecisionMaker::selectStrategy(const bool faultDetected[n_FaultType]){
	if (faultDetected[e_LowBatteryFault] || faultDetected[e_NoMotorCommandFault])
		return CorrectorType::e_NoMotorLowBatteryCorrector;

	if (faultDetected[e_HardZoneFault])
		return CorrectorType::e_HardZoneCorrector;

	if (faultDetected[e_SoftZoneFault])
		return CorrectorType::e_SoftZoneCorrector;
}