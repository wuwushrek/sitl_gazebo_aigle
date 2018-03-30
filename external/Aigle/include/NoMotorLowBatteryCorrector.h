#ifndef __NoMotorLowBatteryCorrector_H__
#define __NoMotorLowBatteryCorrector_H__

#include "FaultCorrector.h"

class NoMotorLowBatteryCorrector : public FaultCorrector {

public:
	NoMotorLowBatteryCorrector(AigleIoInterface *aigleIo);
	virtual void correctFault(const EstimateState * estimate_state) const;
	virtual CorrectorType getCorrectorType() const;
	virtual bool isFaultCorrected(FaultType faultType) const ;
	~NoMotorLowBatteryCorrector();
};

#endif //__NoMotorLowBatteryCorrector_H__