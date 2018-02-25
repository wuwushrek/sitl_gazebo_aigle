#ifndef STRATEGY_H
#define STRATEGY_H

#ifdef AIGLE_SITL_MODE
	#include "io_aigle_sitl.h"
#else
	#include "io_aigle.h"
#endif


class Strategy {

public:

	Strategy(IoAigleInterface *aigle_io);

	void execute_strategy();

	~Strategy();

private:

	MODE check_situation();
	void compute_strategies();

	topology m_topology;
	IoAigleInterface *_aigle_io;
};

#endif //STRATEGY_H