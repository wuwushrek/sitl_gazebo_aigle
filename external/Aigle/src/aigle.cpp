#include "aigle.h"

int main(void ){

	// Initialize aigle io interface with default port and queues elements
	IoAigleInterface aigle_io;
	// Initialize strategy class with aigle_io instance and queues elements
	Strategy strategy(&aigle_io);
	// Initialize estimator class 

	/* Tasks creation */
	BaseType_t status;
	/* Create periodic imu read task */
	status = xTaskCreate(task_imu_read, "imu", STACK_IMU_SIZE, (void *) &aigle_io, SENSOR_READ_PRIORITY, NULL);
	if (status != pdPASS){
		printf("[IMU] COULD_NOT_ALLOCATE_REQUIRED_MEMORY \n");
		return 0;
	}

	/* Create periodic gps read task */
	status = xTaskCreate(task_gps_read, "gps", STACK_GPS_SIZE, (void *) &aigle_io, SENSOR_READ_PRIORITY, NULL);
	if (status != pdPASS){
		printf("[GPS] COULD_NOT_ALLOCATE_REQUIRED_MEMORY \n");
		return 0;
	}

	/* Create strategy control task */
	status = xTaskCreate(task_strategy_control, "strat", STACK_STRAT_SIZE, (void *) &strategy, STRATEGY_PRIORITY, NULL);
	if (status != pdPASS){
		printf("[STRAT] COULD_NOT_ALLOCATE_REQUIRED_MEMORY \n");
		return 0;
	}

	/* Create EKF estimator task */


	#ifdef AIGLE_SITL_MODE
	/* Create periodic gps read task */
	status = xTaskCreate(task_collectSensorsData, "sensor", STACK_IMU_SIZE, (void *) &aigle_io, SENSOR_READ_PRIORITY, NULL);
	if (status != pdPASS){
		printf("[SENSOR] COULD_NOT_ALLOCATE_REQUIRED_MEMORY \n");
		return 0;
	}
	#endif

	/* Start the scheduler itself. */
	vTaskStartScheduler();

	return 0;
}

void task_imu_read(void*  aigle_instance){
	IoAigleInterface *aigle_io = (IoAigleInterface *) aigle_instance;
	TickType_t nextWakeTime;
	const TickType_t periodToTick = pdMS_TO_TICKS(IMU_DEADLINE);
	printf("%s\n", "[Aigle] IMU reading task started");
	nextWakeTime = xTaskGetTickCount();
	while (1){
		vTaskDelayUntil(&nextWakeTime, periodToTick);
		aigle_io->readIMU();
	}
}

void task_gps_read(void*  aigle_instance){
	IoAigleInterface *aigle_io = (IoAigleInterface *) aigle_instance;
	TickType_t nextWakeTime;
	const TickType_t periodToTick = pdMS_TO_TICKS(GPS_DEADLINE);
	printf("%s\n", "[Aigle] GPS reading task started");
	nextWakeTime = xTaskGetTickCount();
	while (1){
		vTaskDelayUntil(&nextWakeTime, periodToTick);
		aigle_io->readGPS();
	}
}

void task_strategy_control(void* strategy_instance){
	Strategy *strategy = (Strategy *) strategy_instance;
	while(1){
		strategy->execute_strategy();
	}
}

void task_ekf_estimator(void* estimator){
	while(1);
}

#ifdef AIGLE_SITL_MODE
void task_collectSensorsData(void* aigle_instance){
	IoAigleInterface *aigle_io = (IoAigleInterface *) aigle_instance;
	TickType_t nextWakeTime;
	const TickType_t periodToTick = pdMS_TO_TICKS(SENSORS_DEADLINE);
	printf("%s\n", "[Aigle] Sensors reading task started");
	nextWakeTime = xTaskGetTickCount();
	while (1){
		vTaskDelayUntil(&nextWakeTime, periodToTick);
		aigle_io->collectSensorsData();
	}
}
#endif

/*
* FreeRTOS that must be implemented in order to compile
*/
void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{
    printf("ASSERT: %s : %d\n", pcFileName, (int)ulLine);
    while(1);
}


unsigned long ulGetRunTimeCounterValue(void)
{
    return 0;
}

void vConfigureTimerForRunTimeStats(void)
{
    return;
}


void vApplicationMallocFailedHook(void)
{
	while(1);
}