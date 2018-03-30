#ifndef __FaultDetector_H__
#define __FaultDetector_H__

#include "AigleTypes.h"

class FaultDetector {

protected:
	FaultType faultID;

public:
	virtual bool detectFault(const EstimateState * estimate_state) const = 0;
	virtual FaultType getFaultType() const  = 0;
};

#endif // __FaultDetector_H__