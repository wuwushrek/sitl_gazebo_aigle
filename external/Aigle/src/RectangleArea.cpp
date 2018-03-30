#include "RectangleArea.h"
#include <stdio.h>

RectangleArea::RectangleArea(float x_min , float x_max , float y_min , int y_max){
	this->x_min = x_min;
	this->y_min = y_min;
	this->x_max = x_max;
	this->y_max = y_max;
}

RectangleArea::~RectangleArea(){
	printf("Suppression RectangleArea \n");
}

bool RectangleArea::outOfArea(const EstimateState *m_estimate) const {
	return ! (m_estimate->x > x_min && m_estimate->x < x_max && m_estimate->y > y_min && m_estimate->y < y_max);
}