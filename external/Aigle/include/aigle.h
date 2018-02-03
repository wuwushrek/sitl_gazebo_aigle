#ifndef AIGLE_H
#define AIGLE_H

#ifdef AIGLE_SITL_MODE
	#include "io_aigle_sitl.h"
#else
	#include "io_aigle.h"
#endif

#include "strategy.h"
#include <iostream>


IoAigleInterface aigle_io;
Strategy strategy;

#ifndef AIGLE_SITL_MODE
uint8_t init_aigle();
#else
uint8_t init_aigle(int port_to_gazebo);
#endif

void execute_strategy();


#endif //AIGLE_H