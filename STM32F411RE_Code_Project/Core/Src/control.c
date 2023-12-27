#include <control.h>
#include "tim.h"
#include <math.h>
#include <stdbool.h>
#include "i2c.h"
#include <stdint.h>



void PID_Init(PID_Param_t* PID){
	PID -> Kp = 1.8410;
	PID -> Ki = 0.0118;
	PID -> Kd = 60.8833;
	PID -> deltaT = 1;
	PID -> derivative = 0;
	PID -> integral = 0;
	PID -> current_error = 0;
	PID -> current_output = 0;
	PID -> prev_error = 0;
	PID -> prev_output = 0;
	PID -> saturation = false;

}

void Pump_Init(Pump_t* pump){
	pump -> PowerCurrent = 0;
}

void Tank1_Init(Tank_Param_t* tank){
	tank -> height = 25.5;
	tank -> level_current = 0;
	tank -> level_target = 0;
	tank -> sensor_height = 29.5;
}

void Tank2_Init(Tank_Param_t* tank){
	tank -> height = 25.5;
	tank -> level_current = 0;
	tank -> level_target = 10;
	tank -> sensor_height = 29.5;
}

void LQI_Init(LQI_Param_t* lqi){
	lqi -> K1 = -15.7345;
	lqi -> K2 = -23.1724;
	lqi -> K3 = 0.3162;
	lqi -> current_error = 0;
	lqi -> current_output = 0;
	lqi -> deltaT = 1;
	lqi -> prev_error = 0;
	lqi -> prev_output = 0;
	lqi -> saturation = false;
}


float PID_Control(PID_Param_t* PID, Tank_Param_t* tank){
	float epsilon = 0.1;
	PID -> prev_output = PID -> current_output;
	PID -> current_error = -tank->level_current + tank->level_target;
	if(fabsf(PID -> current_error) > epsilon){
		PID -> integral += PID -> current_error*PID -> deltaT;
		PID -> derivative = (PID -> current_error - PID -> prev_error)/PID->deltaT;
		PID -> current_output = PID->Kp*PID->current_error + PID->Ki*PID ->integral + PID->Kd*PID->derivative;
		if(PID -> current_output > 100){
			PID -> saturation = true;
			PID -> current_output = 100;
		}
		else if(PID -> current_output < 0){
			PID -> saturation = true;
			PID -> current_output = 0;
		}
		else{
			PID -> saturation = false;
		}
	}
	PID -> prev_error = PID -> current_error;

	return PID ->current_output;
}

/*
float LQR_Control(LQR_Param_t* LQR, Tank_Param_t* tank1, Tank_Param_t* tank2){
	float epsilon = 1;
	LQR -> prev_output = LQR -> current_output;
	LQR->current_error = tank1->level_current*LQR->K1 + tank2->level_current*LQR->K2 - tank2->level_target;
	if(fabsf(LQR -> current_error) > epsilon){
		LQR ->current_output = LQR->current_error;
		if(LQR -> current_output > 100){
			LQR -> saturation = true;
			LQR -> current_output = 100;
		}
		else if(LQR -> current_output < 0){
			LQR -> saturation = true;
			LQR -> current_output = 0;
		}
		else{
			LQR -> saturation = false;
		}

	}
	LQR -> prev_error = LQR -> current_error;
	return LQR ->current_output;

} */

float LQI_Control(LQI_Param_t* LQI, Tank_Param_t* tank1, Tank_Param_t* tank2 ){
	float epsilon = 0.1;
	LQI -> prev_output = LQI -> current_output;
	LQI -> current_error = -tank2->level_current +tank2->level_target;
	if(fabsf(LQI -> current_error) > epsilon){
		LQI -> integral += LQI -> current_error*LQI -> deltaT;
		LQI ->current_output = + tank1->level_current*LQI->K1 + tank2->level_current*LQI->K2 + LQI->K3*LQI->integral;
		if(LQI -> current_output > 100){
			LQI -> saturation = true;
			LQI -> current_output = 100;
		}
		else if(LQI -> current_output < 0){
			LQI -> saturation = true;
			LQI -> current_output = 0;
		}
		else{
			LQI -> saturation = false;
		}

	}
	LQI -> prev_error = LQI -> current_error;
	return LQI ->current_output;

}

void actuation_pump(Pump_t* pump, float DC){

	pump->PowerCurrent = (DC);
	if(pump->PowerCurrent > 0){
		TIM2->CCR1 = ((pump->PowerCurrent)/100)*(4199 + 1);
	}
	else{
		TIM2->CCR1 = 0;
	}
}

void MapLevel(uint32_t *data1,uint32_t *data2,Tank_Param_t* tank1,Tank_Param_t* tank2,float* lev1,float* lev2)
{
	*lev1 = tank1->sensor_height - (float)(*data1)/10;
	*lev2 = tank2->sensor_height - (float)(*data2)/10;
}




