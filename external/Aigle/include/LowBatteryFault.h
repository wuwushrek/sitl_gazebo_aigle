#ifndef __LowBatteryFault_H__
#define __LowBatteryFault_H__ 

#include "FaultDetector.h"

class LowBatteryFault : public FaultDetector {
public:
	LowBatteryFault(uint8_t min_bat);
	virtual bool detectFault(const EstimateState * estimate_state) const;
	virtual FaultType getFaultType() const;
	~LowBatteryFault();

private:
	uint8_t min_battery_level;
};

#endif //__LowBatteryFault_H__