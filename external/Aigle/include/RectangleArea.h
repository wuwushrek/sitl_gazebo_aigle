#ifndef __RECTANGLEAREA_H__
#define __RECTANGLEAREA_H__

#include "Area.h"

class RectangleArea : public Area{

public:
	RectangleArea(float x_min , float x_max , float y_min , int y_max);
	virtual bool outOfArea(const EstimateState *m_estimate) const;
	~RectangleArea();
	
private:
	int x_min;
	int x_max;
	int y_min;
	int y_max;
};

#endif //__RECTANGLEAREA_H__