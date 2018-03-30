#ifndef __OutOfZoneCorrector_H__
#define __OutOfZoneCorrector_H__

#include "FaultCorrector.h"

#define V_REPLI 2.0f
#define K3_const 0.1f
#define K2_const 5.0f
#define K1_const 2.0f

class InsideHardZoneCorrector : public FaultCorrector {

public:
	InsideHardZoneCorrector(AigleIoInterface *aigleIo);
	virtual void correctFault(const EstimateState * estimate_state) const;
	virtual CorrectorType getCorrectorType() const;
	virtual bool isFaultCorrected(FaultType faultType) const;
	~InsideHardZoneCorrector();
};

class InsideSoftZoneCorrector : public FaultCorrector {

public:
	InsideSoftZoneCorrector(AigleIoInterface *aigleIo);
	virtual void correctFault(const EstimateState * estimate_state) const;
	virtual CorrectorType getCorrectorType() const;
	virtual bool isFaultCorrected(FaultType faultType) const;
	~InsideSoftZoneCorrector();
};

#endif //__OutOfZoneCorrector_H__