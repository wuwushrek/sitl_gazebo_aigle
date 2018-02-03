#ifndef STRATEGY_H
#define STRATEGY_H

#include "aigle_types.h"

#include <stdio.h>
#include <string.h>

class Strategy {

public:

	Strategy();

	MODE check_situation(const gps_data *current_gps_data);
	void compute_strategies(const gps_data *current_gps_data);
	const motor_data* get_strategy_motors_data();

	~Strategy();

private:
	topology m_topology;
	motor_data strategy_motors_data;
};

#endif //STRATEGY_H