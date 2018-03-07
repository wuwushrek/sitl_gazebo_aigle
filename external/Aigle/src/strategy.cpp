
#include "strategy.h"

Strategy::Strategy(IoAigleInterface *aigle_io){
	_aigle_io = aigle_io;
	m_position = {0};
}

Strategy::~Strategy(){}

void Strategy::execute_strategy(){
	xQueueReceive(estim_queue , &m_position , ( TickType_t ) 0);

	MODE situation = check_situation();
	if (situation == MODE_SAFE){
		// std::cout << "Mode safe" << std::endl;
		_aigle_io->transferMotorsValue();
	} else{
		compute_strategies();
	}
}


void Strategy::setTopology(topology topo){
	m_topology = topo;
}


void Strategy::setEstimatePosQueue(QueueHandle_t estimQueue){
	estim_queue =  estimQueue;
}

MODE Strategy::check_situation(){
	if (m_position.x > m_topology.x_max || m_position.x < m_topology.x_min || m_position.y > m_topology.y_max || m_position.y < m_topology.y_min ){
		return MODE_UNSAFE;
	}else{
		return MODE_SAFE;
	}
}

void Strategy::compute_strategies(){
	motor_data m_motor;
	m_motor.is_armed = 0;
	for (uint8_t i = 0 ; i< n_out_max ; i++){
		m_motor.motor_values[i] = 1500;
	}
	_aigle_io->writeMotors(&m_motor);
}	
