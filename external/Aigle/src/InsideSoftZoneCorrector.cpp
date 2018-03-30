#include "OutOfZoneCorrector.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"

#define EPS_POS 1.0f
#define EPS_ANGLE 0.25f


InsideSoftZoneCorrector::InsideSoftZoneCorrector(AigleIoInterface *aigleIo){
	this->aigleIo = aigleIo;
	correctorID = CorrectorType::e_SoftZoneCorrector;
	for (uint8_t i =0 ; i< n_FaultType ; i++){
		correctedFaults[i] = false;
	}
	correctedFaults[FaultType::e_SoftZoneFault] =  true;
}

InsideSoftZoneCorrector::~InsideSoftZoneCorrector(){
	printf("Inside Soft zone corrector deleted ! \n");
}

CorrectorType InsideSoftZoneCorrector::getCorrectorType() const {
	return this->correctorID;
}

bool InsideSoftZoneCorrector::isFaultCorrected(FaultType faultType) const{
	return correctedFaults[faultType];
}

void InsideSoftZoneCorrector::correctFault(const EstimateState *estimate_state) const{
	float v , gamma;
	if (fabs(estimate_state->y) < EPS_POS && fabs(estimate_state->yaw) < EPS_ANGLE ){
		v = - K3_const * estimate_state->x;
		v = fabs(v) > 0.35 ? v/fabs(v) * 0.35 : v;
		gamma = 0;
	}else{
		float alpha = atan2(estimate_state->y , estimate_state->x);
		if (cos(alpha - estimate_state->yaw) >0 ){
			v = - V_REPLI;
		} else{
			v = V_REPLI;
		}
		v= V_REPLI;
		float sinyaw_yaw;
		if (fabs(estimate_state->yaw) < 1.0e-8f){
			sinyaw_yaw =  0;
		}else{
			sinyaw_yaw = sin(estimate_state->yaw) / estimate_state->yaw;
		}
		gamma = atan(-(K2_const*estimate_state->yaw + K1_const*v*sinyaw_yaw*estimate_state->y)/v);
		v = ( fabs(v)/ v ) * 0.45f;
	}
	MotorData outputMotors={0};
	outputMotors.is_armed = 1;
	for (uint8_t i= 0; i< n_out_max ; i++){
		outputMotors.motor_values[i] = 0.0f;
	}
	outputMotors.motor_values[0] = (fabs(gamma) > 0.5235  ? (gamma/fabs(gamma)) * 1.0f : gamma/0.5235)*0.2;
	outputMotors.motor_values[1] = outputMotors.motor_values[0];
	outputMotors.motor_values[5] = v;
	outputMotors.motor_values[6] = outputMotors.motor_values[5];
	this->aigleIo->writeMotors(&outputMotors);
	// printf("%f , %f \n", outputMotors.motor_values[0] , outputMotors.motor_values[5] );
}