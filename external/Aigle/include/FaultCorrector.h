#ifndef __FaultCorrector_H__
#define __FaultCorrector_H__

#include "AigleTypes.h"
#include "AigleIoInterface.h"

class FaultCorrector {

protected:
	CorrectorType correctorID;
	bool correctedFaults[n_FaultType];
	AigleIoInterface *aigleIo;
public:
	virtual void correctFault(const EstimateState * estimate_state) const = 0;
	virtual CorrectorType getCorrectorType() const  = 0;
	virtual bool isFaultCorrected(FaultType faultType) const = 0;
};

#endif // __FaultCorrector_H__