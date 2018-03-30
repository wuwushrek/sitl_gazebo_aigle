#ifndef __OutOfZoneFault_H__
#define __OutOfZoneFault_H__ 

#include "FaultDetector.h"
#include "Area.h"

class OutOfZoneFault : public FaultDetector {
public:
	OutOfZoneFault(Area *area , FaultType faultID);
	virtual bool detectFault(const EstimateState * estimate_state) const;
	virtual FaultType getFaultType() const;
	~OutOfZoneFault();

private:
	Area *area;
};

class InsideSoftZoneFault : public OutOfZoneFault{
public:
	InsideSoftZoneFault(Area *area) : OutOfZoneFault(area,FaultType::e_SoftZoneFault)
	{}
};

class InsideHardZoneFault : public OutOfZoneFault{
public:
	InsideHardZoneFault(Area *area) : OutOfZoneFault(area,FaultType::e_HardZoneFault)
	{}
};

#endif //__OutOfZoneFault_H__