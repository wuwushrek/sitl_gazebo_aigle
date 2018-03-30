#ifndef __AREA_H__
#define __AREA_H__

#include "AigleTypes.h"

class Area {
public:
	virtual bool outOfArea(const EstimateState *m_estimate) const = 0;
};

#endif //__AREA_H__