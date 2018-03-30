#ifndef __StrategyDecisionMaker_H__
#define __StrategyDecisionMaker_H__

#include "AigleTypes.h"

class StrategyDecisionMaker {

public:
	StrategyDecisionMaker();
	CorrectorType selectStrategy(const bool faultDetected[n_FaultType]);
	~StrategyDecisionMaker();
};

#endif //__StrategyDecisionMaker_H__