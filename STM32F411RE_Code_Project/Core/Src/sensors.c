/*
 * sensors.c
 *
 *  Created on: Mar 24, 2023
 *      Author: anto-
 */

#include <sensors.h>
#include "gpio.h"
#include "i2c.h"
#include "vl53l0x_api.h"
#include "stdio.h"


uint16_t arr_gpio[2] = {X_SHUT_1_Pin,X_SHUT_2_Pin};
uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;


void Sensor_Init(struct sensors s,sensor_modes mode){

	HAL_GPIO_WritePin(GPIOC, arr_gpio[0], GPIO_PIN_RESET); 	// Disable XSHUT
	HAL_GPIO_WritePin(GPIOC, arr_gpio[1], GPIO_PIN_RESET); 	// Disable XSHUT
	HAL_Delay(10);

	for (int i = 0; i<2;i++){

		HAL_GPIO_WritePin(GPIOC, arr_gpio[i], GPIO_PIN_SET); 		// Enable XSHUT
		HAL_Delay(20);

		VL53L0X_WaitDeviceBooted(s.Dev_array[i]);
		VL53L0X_DataInit((s.Dev_array[i]));
		if (i == 0){
			printf("Addr change 1: %i \n\r\n\r", VL53L0X_SetDeviceAddress(s.Dev_array[i], sensor1_addr));
			s.Dev_array[i]->I2cDevAddr = sensor1_addr;
		}else{
			printf("Addr change 2: %i \n\r\n\r", VL53L0X_SetDeviceAddress(s.Dev_array[i], sensor2_addr));
			s.Dev_array[i]->I2cDevAddr = sensor2_addr;
		}
		Sensor_Config(s.Dev_array[i],mode);
	}
}


void Sensor_Config(VL53L0X_DEV Dev,sensor_modes mode){

	VL53L0X_WaitDeviceBooted( Dev );
	VL53L0X_StaticInit( Dev );
	VL53L0X_PerformRefCalibration(Dev,  &VhvSettings, &PhaseCal);
	VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
	VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

	switch(mode){
					case 0:
						printf("LONG RANGE");
						VL53L0X_SetLimitCheckValue(Dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.1*65536));
						VL53L0X_SetLimitCheckValue(Dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(60*65536));
						VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev,33000);
						VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
						VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
						break;
					case 1:
						printf("HIGH SPEED");
						VL53L0X_SetLimitCheckValue(Dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.25*65536));
						VL53L0X_SetLimitCheckValue(Dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(32*65536));
						VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev,	20000);
						break;
					case 2:
						printf("HIGH ACCURACY");
						VL53L0X_SetLimitCheckValue(Dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
						(FixPoint1616_t)(0.25*65536));
						VL53L0X_SetLimitCheckValue(Dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(18*65536));
						VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev,200000);
						break;

	}
	HAL_Delay(100);
}


void Get_Data_Sensors(struct sensors s, uint32_t* data1,uint32_t* data2){

		/*Check that these are not pointing to NULL*/
		assert(data1);
		assert(data2);

		if((VL53L0X_PerformSingleRangingMeasurement(s.Dev_array[0], &s.RangingData[0]) == 0) ){

			if( (s.RangingData[0].RangeStatus == 0)  ){
				*data1 = s.RangingData[0].RangeMilliMeter;

			}
			else{
				printf("STATUS1: %i\n\r",  s.RangingData[0].RangeStatus);
			}
		}

		if((VL53L0X_PerformSingleRangingMeasurement(s.Dev_array[1], &s.RangingData[1]) == 0) ){

				if( (s.RangingData[1].RangeStatus == 0)  ){
					*data2 = s.RangingData[1].RangeMilliMeter;
				}
				else{
					printf("STATUS2: %i\n\r", s.RangingData[1].RangeStatus);
				}
			}


}

