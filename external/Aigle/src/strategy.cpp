
#include "strategy.h"

Strategy::Strategy(){

}

Strategy::~Strategy(){
}

MODE Strategy::check_situation(const gps_data *current_gps_data){
	return MODE_SAFE;
}

void Strategy::compute_strategies(const gps_data *current_gps_data){

}

const motor_data* Strategy::get_strategy_motors_data(){
	return &strategy_motors_data;
}