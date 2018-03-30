#ifndef __StrategyManager_H__
#define __StrategyManager_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdint.h>
#include "AigleIoInterface.h"
#include "FaultCorrector.h"
#include "FaultDetector.h"
#include "StrategyDecisionMaker.h"

class StrategyManager {

private:
	AigleIoInterface *aigle_io;
	QueueHandle_t estimateQueueHandle;
	EstimateState current_state;

	FaultDetector* listFaultDetector[MAX_DETECTOR];
	FaultCorrector* listFaultCorrector[n_FaultCorrector];

	StrategyDecisionMaker decisionMaker;

	uint8_t max_detectors;
	bool first_iteration;

	bool detectFailure(bool faultDetected[n_FaultType]);
	CorrectorType selectStrategy(const bool faultDetected[n_FaultType]);
	bool updateEstimateState();

public:
	StrategyManager(AigleIoInterface *aigle_io);
	void setEstimateStateQueue(QueueHandle_t estimateQueueHandle_);
	void detectAndApplyStrategy();
	bool addFaultDetector(FaultDetector *fautDect);
	bool addFaultCorrector(FaultCorrector *faultCorr);
	bool isAllFaultCorrected();
	~StrategyManager();

};

#endif //__StrategyManager_H__