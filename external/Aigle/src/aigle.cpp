
#include "aigle.h"

#ifdef AIGLE_SITL_MODE
uint8_t init_aigle(int port_to_gazebo){
	aigle_io = IoAigleInterface(port_to_gazebo);
#else
uint8_t init_aigle(){
	aigle_io =  IoAigleInterface();
#endif
	if (aigle_io.succeed_init != 1)
		return aigle_io.succeed_init;
	// Initialize the strategy to implement
	strategy = Strategy();
	return 1;
}

void execute_strategy(){

	// try to update sensors values
	aigle_io.readMotors();
	strategy.compute_strategies(NULL); // TODO

	MODE situation = strategy.check_situation(NULL); //TO MODIFY ACCORDING TO current GPS data
	
	if (situation == MODE_SAFE && aigle_io.new_data){
		aigle_io.writeMotors(aigle_io.get_motor_data());
	}/*else {
		aigle_io.writeMotors(strategy.get_strategy_motors_data());
	}*/
}

/*int main(){

	if( init_aigle(14570) != 1){
		std::cout << "Failed to initialize AIGLE !!" << std::endl;
		return 0;
	}
	while(1){
		execute_strategy();
	}
	return 0;
}*/