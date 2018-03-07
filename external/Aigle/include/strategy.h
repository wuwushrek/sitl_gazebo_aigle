#ifndef STRATEGY_H
#define STRATEGY_H

#ifdef AIGLE_SITL_MODE
	#include "io_aigle_sitl.h"
#else
	#include "io_aigle.h"
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

class Strategy {

public:

	Strategy(IoAigleInterface *aigle_io);

	void execute_strategy();
	void setTopology(topology topo);
	void setEstimatePosQueue(QueueHandle_t estimQueue);

	~Strategy();

private:

	MODE check_situation();
	void compute_strategies();

	topology m_topology;
	IoAigleInterface *_aigle_io;

	QueueHandle_t estim_queue;
	estimate_position m_position;
};

#endif //STRATEGY_H