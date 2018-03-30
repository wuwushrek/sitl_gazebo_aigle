#include "aigle.h"
#include "LowBatteryFault.h"
#include "NoMotorCommandFault.h"
#include "NoMotorLowBatteryCorrector.h"
#include "RectangleArea.h"
#include "OutOfZoneFault.h"
#include "OutOfZoneCorrector.h"

#define MIN_BATTERY_LEVEL 10
#define MIN_DELAY_MOTOR 100

int main(void ){

	// Create queue for message exchange between some tasks
	QueueHandle_t gpsQueueHandle, imuQueueHandle, estQueueHandle,ssQueueHandle;
	gpsQueueHandle = xQueueCreate(1 , sizeof (GpsData));
	imuQueueHandle = xQueueCreate(1 , sizeof (ImuData));
	estQueueHandle = xQueueCreate(1 , sizeof (EstimateState));
	ssQueueHandle = xQueueCreate(1, sizeof (SystemStatus));

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
		printf("[AIGLE] Could not create est state queue \n");
		return 0;
	}
	if(ssQueueHandle == NULL){
		printf("[AIGLE] Could not create system status queue \n");
		return 0;
	}

	// Initialize aigle io interface with default port and queues elements
	AigleIoInterface *aigle_io = NULL;
	#ifdef AIGLE_SITL_MODE
		AigleIoInterfaceSITL aigle_io_sitl;
		aigle_io = &aigle_io_sitl;
	#else
		AigleIoInterfaceReal aigle_io_real;
		aigle_io = &aigle_io_real;
	#endif
	aigle_io->setGPSQueue(gpsQueueHandle);
	aigle_io->setIMUQueue(imuQueueHandle);
	aigle_io->setSystemStatusQueue(ssQueueHandle);

	// Initialize strategy class with aigle_io instance and queues elements
	StrategyManager strategy(aigle_io);
	strategy.setEstimateStateQueue(estQueueHandle);

	LowBatteryFault lowBattery(MIN_BATTERY_LEVEL);
	NoMotorCommandFault noMotorCommand(MIN_DELAY_MOTOR);
	RectangleArea soft_area(-10 , 10 , -10 , 10);
	RectangleArea hard_area(-30 , 30 , -30 , 30);
	OutOfZoneFault soft_fault =InsideSoftZoneFault((Area *) &soft_area);
	OutOfZoneFault hard_fault = InsideHardZoneFault((Area *) &hard_area);
	strategy.addFaultDetector((FaultDetector *) &lowBattery);
	strategy.addFaultDetector((FaultDetector *) &noMotorCommand);
	strategy.addFaultDetector((FaultDetector *) &soft_fault);
	strategy.addFaultDetector((FaultDetector *) &hard_fault);

	NoMotorLowBatteryCorrector LMCorrector(aigle_io);
	InsideSoftZoneCorrector soft_corrector(aigle_io);
	InsideHardZoneCorrector hard_corrector(aigle_io);
	strategy.addFaultCorrector((FaultCorrector *) &LMCorrector);
	strategy.addFaultCorrector((FaultCorrector *) &soft_corrector);
	strategy.addFaultCorrector((FaultCorrector *) &hard_corrector);

	if (strategy.isAllFaultCorrected()){
		printf("All fault has at least one corrector ! \n");
	} else {
		printf("Some detected fault doesn't have a corrector ! \n");
		return 0;
	}
	// Initialize estimator class 
	Estimator estimator;
	estimator.setGPSQueue(gpsQueueHandle);
	estimator.setIMUQueue(imuQueueHandle);
	estimator.setEstimateStateQueue(estQueueHandle);
	estimator.setSystemStatusQueue(ssQueueHandle);

	/* Tasks creation */
	BaseType_t status;

	/* Create periodic imu read task */
	status = xTaskCreate(task_imu_read, "imu", STACK_IMU_SIZE, (void *) aigle_io, SENSOR_READ_PRIORITY, NULL);
	if (status != pdPASS){
		printf("[IMU] COULD_NOT_ALLOCATE_REQUIRED_MEMORY \n");
		return 0;
	}

	/* Create periodic gps read task */
	status = xTaskCreate(task_gps_read, "gps", STACK_GPS_SIZE, (void *) aigle_io, SENSOR_READ_PRIORITY, NULL);
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
	status = xTaskCreate(task_collectSensorsData, "sensor", STACK_IMU_SIZE, (void *) aigle_io, SENSOR_READ_PRIORITY, NULL);
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
	AigleIoInterface *aigle_io = (AigleIoInterface *) aigle_instance;
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
	AigleIoInterface *aigle_io = (AigleIoInterface *) aigle_instance;
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
	StrategyManager *strategy = (StrategyManager *) strategy_instance;
	TickType_t nextWakeTime;
	const TickType_t periodToTick = pdMS_TO_TICKS(STRAT_DEADLINE);
	printf("%s\n", "[STRAT] Stratedy task started");
	nextWakeTime = xTaskGetTickCount();
	while(1){
		vTaskDelayUntil(&nextWakeTime, periodToTick);
		strategy->detectAndApplyStrategy();
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
	AigleIoInterface *aigle_io = (AigleIoInterface *) aigle_instance;
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