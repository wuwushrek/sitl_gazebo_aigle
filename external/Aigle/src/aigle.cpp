#include "aigle.h"

#define X_MIN -36
#define Y_MIN -36
#define X_MAX 36
#define Y_MAX 36

int main(void ){
	topology m_limit ={0};
	m_limit.x_min = X_MIN;
	m_limit.x_max = X_MAX;
	m_limit.y_min = Y_MIN;
	m_limit.y_max = Y_MAX;

	// Create queue for message exchange between some tasks
	QueueHandle_t gpsQueueHandle, imuQueueHandle, estQueueHandle;
	gpsQueueHandle = xQueueCreate(1 , sizeof (gps_data));
	imuQueueHandle = xQueueCreate(1 , sizeof (imu_data));
	estQueueHandle = xQueueCreate(1 , sizeof (estimate_position));

	// Check if creattion was done
	if (gpsQueueHandle == NULL){
		printf ("[AIGLE] Could not create gps queue \n");
		return 0;
	}
	if(imuQueueHandle == NULL){
		printf("[AIGLE] Could not create imu queue \n");
		return 0;
	}
	if(estQueueHandle == NULL){
		printf("[AIGLE] Could not create estimpos queue \n");
		return 0;
	}

	// Initialize aigle io interface with default port and queues elements
	IoAigleInterface aigle_io;
	aigle_io.setGPSQueue(gpsQueueHandle);
	aigle_io.setIMUQueue(imuQueueHandle);

	// Initialize strategy class with aigle_io instance and queues elements
	Strategy strategy(&aigle_io);
	strategy.setTopology(m_limit);
	strategy.setEstimatePosQueue(estQueueHandle);

	// Initialize estimator class 
	Estimator estimator;
	estimator.setGPSQueue(gpsQueueHandle);
	estimator.setIMUQueue(imuQueueHandle);
	estimator.setEstimatePosQueue(estQueueHandle);

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
	status = xTaskCreate(task_ekf_estimator, "ekf", STACK_ESTIM_SIZE, (void *) &estimator, ESTIMATOR_PRIORITY, NULL);
	if (status != pdPASS){
		printf("[EKF] COULD_NOT_ALLOCATE_REQUIRED_MEMORY \n");
		return 0;
	}

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
	TickType_t nextWakeTime;
	const TickType_t periodToTick = pdMS_TO_TICKS(STRAT_DEADLINE);
	printf("%s\n", "[STRAT] Stratedy task started");
	nextWakeTime = xTaskGetTickCount();
	while(1){
		vTaskDelayUntil(&nextWakeTime, periodToTick);
		strategy->execute_strategy();
	}
}

void task_ekf_estimator(void* estim){
	Estimator *estimator = (Estimator *) estim;
	printf("%s\n", "[EKF] EKF estimator task started");
	while(1){
		estimator->update();
	}
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