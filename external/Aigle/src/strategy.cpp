
#include "strategy.h"

Strategy::Strategy(IoAigleInterface *aigle_io){
	_aigle_io = aigle_io;
}

Strategy::~Strategy(){}

void Strategy::execute_strategy(){
	MODE situation = check_situation();
	if (situation == MODE_SAFE){
		_aigle_io->transferMotorsValue();
	} else{
		compute_strategies();
	}
}

MODE Strategy::check_situation(){
	return MODE_SAFE;
}

void Strategy::compute_strategies(){

}
