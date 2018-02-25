#ifndef AIGLE_H
#define AIGLE_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <iostream>

#ifdef AIGLE_SITL_MODE
	#include "io_aigle_sitl.h"
#else
	#include "io_aigle.h"
#endif

#include "strategy.h"

/* Deadlines are in millisecond */
#ifdef AIGLE_SITL_MODE
	#define SENSORS_DEADLINE 10UL
	#define IMU_DEADLINE 10UL
	#define GPS_DEADLINE 100UL
#else
	#define IMU_DEADLINE 8UL
	#define GPS_DEADLINE 100UL
#endif
/********************************/

/* Define Task priorities									*/
#define SENSOR_READ_PRIORITY 	( configMAX_PRIORITIES - 1 )
#define STRATEGY_PRIORITY 		( configMAX_PRIORITIES - 2 )
#define ESTIMATOR_PRIORITY		( configMAX_PRIORITIES - 3 )		
/************************************************************/

/* Define Task stack size (Need (a lot of) improvements) 				*/
#ifdef AIGLE_SITL_MODE
	#define STACK_IMU_SIZE			100
	#define STACK_GPS_SIZE			100
	#define STACK_STRAT_SIZE		1000
	#define STACK_ESTIM_SIZE		1000
#else
	#define STACK_IMU_SIZE			50
	#define STACK_GPS_SIZE			50
	#define STACK_STRAT_SIZE		1000
	#define STACK_ESTIM_SIZE		1000
#endif
/************************************************************/

/*
* Task main function for reading IMU data 
* This is a periodic task with the period defined by IMU_DEADLINE
* This task has the high priority within all the tasks
*/
void task_imu_read(void*  aigle_instance);

/*
* Task main function for reading GPS data 
* This is a periodic task with the period defined by GPS_DEADLINE
* This task has the high priority within all the tasks
*/
void task_gps_read(void*  aigle_instance);

/*
* Task main function for deciding when to transfer PX4 motor value or not
* This is a non periodic task that apply the strategy then the control of
* the motors. **TODO : Separate actuator output and Strategy using a task**
* This task has the second high priority
*/
void task_strategy_control(void* strategy);

/*
* Task main function for EKF estimation of position/velocity and heading 
* This is a non periodic task than can be preempted by all others tasks
* SO This task has the lowest priority
*/
void task_ekf_estimator(void* estimator);


#ifdef AIGLE_SITL_MODE
/*
* Task main function for reading IMU,GPS and motors data 
* This is a periodic task with the period defined by SENSORS_DEADLINE
* This task has the high priority within all the tasks
* This is only use in a simulation context because all sensors data
* come from the same socket so we need to dispatch them to simulate real sensors
*/
void task_collectSensorsData(void* aigle_instance);
#endif

#endif //AIGLE_H