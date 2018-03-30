#ifndef __NoMotorCommandFault_H__
#define __NoMotorCommandFault_H__ 

#include "FreeRTOS.h"
#include "task.h"

#include <cstdint>
#include "FaultDetector.h"

class NoMotorCommandFault : public FaultDetector {

public:
	NoMotorCommandFault(uint64_t min_delay);
	virtual bool detectFault(const EstimateState * estimate_state) const;
	virtual FaultType getFaultType() const;
	~NoMotorCommandFault();

private:
	uint64_t min_delay;
};

#endif //__LowBatteryFault_H__