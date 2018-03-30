#include "StrategyManager.h"
#include <iostream>
StrategyManager::StrategyManager(AigleIoInterface *aigle_io_){
	max_detectors = 0;
	decisionMaker = StrategyDecisionMaker();
	for (uint8_t i = 0 ; i< MAX_DETECTOR;i++){
		listFaultDetector[i] = NULL;
	}
	for(uint8_t i = 0 ; i< n_FaultCorrector;i++){
		listFaultCorrector[i] = NULL;
	}
	aigle_io = aigle_io_;
	first_iteration = true;
}

StrategyManager::~StrategyManager(){

}

bool StrategyManager::detectFailure(bool faultDetected[n_FaultType]){
	uint8_t i;
	bool res = false;
	for(i=0 ; i< n_FaultType; i++){
		faultDetected[i] = false;
	}
	for(i=0 ; i< max_detectors ; i++){
		faultDetected[listFaultDetector[i]->getFaultType()] |= listFaultDetector[i]->detectFault(&current_state);
		res |= faultDetected[listFaultDetector[i]->getFaultType()];
	}
	return res;
}

CorrectorType StrategyManager::selectStrategy(const bool faultDetected[n_FaultType]){
	return static_cast<CorrectorType>(decisionMaker.selectStrategy(faultDetected));
}

bool StrategyManager::updateEstimateState(){
	return xQueueReceive(estimateQueueHandle , &current_state , ( TickType_t ) 0);
}

void StrategyManager::setEstimateStateQueue(QueueHandle_t estimateQueueHandle_){
	estimateQueueHandle = estimateQueueHandle_;
}

bool StrategyManager::addFaultDetector(FaultDetector *faultDect){
	if (max_detectors == MAX_DETECTOR-1)
		return false;
	listFaultDetector[max_detectors] = faultDect;
	max_detectors++;
	return true;
}

bool StrategyManager::addFaultCorrector(FaultCorrector *faultCorr){
	listFaultCorrector[faultCorr->getCorrectorType()] = faultCorr;
	return true;
}

bool StrategyManager::isAllFaultCorrected(){
	bool res = true;
	for(uint8_t i = 0 ; i< max_detectors ; i++){
		bool is_corr = false;
		for(uint8_t j = 0 ; j<n_FaultCorrector ; j++){
			if (listFaultCorrector[j] == NULL)
				continue;
			printf("%d , %d\n",listFaultDetector[i]->getFaultType(),listFaultCorrector[j]->getCorrectorType());
			is_corr |= listFaultCorrector[j]->isFaultCorrected(listFaultDetector[i]->getFaultType());
		}
		res &= is_corr;
	}
	return res;
}

void StrategyManager::detectAndApplyStrategy(){
	if (! updateEstimateState() && first_iteration){
		return;
	}else{
		first_iteration = false;
	}
	// std::cout << current_state.x << " ; " << current_state.y << " ; " << current_state.z <<" ; " << current_state.yaw << " ; " << (int) current_state.battery_level << " ; " << current_state.last_time_motors_received << std::endl;
	bool faultDetected[n_FaultType];
	bool hasFault = detectFailure(faultDetected);
	if (hasFault){
		CorrectorType corrector = selectStrategy(faultDetected);
		listFaultCorrector[corrector]->correctFault(&current_state);
		// std::cout << "Correction applied : " << corrector << std::endl;
	}else{
		this->aigle_io->transferMotorsValue();
	}
}