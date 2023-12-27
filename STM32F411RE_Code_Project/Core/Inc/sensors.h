/*
 * sensors.h
 *
 *  Created on: Mar 24, 2023
 *      Author: anto-
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "vl53l0x_api.h"
#include "stdio.h"
#include "i2c.h"
#include "gpio.h"


#define VL53L0x_SINGLE_DEVICE_DRIVER 0
#define sensor_qty 2			// max. 2

#define original_addr 	0x52
#define sensor1_addr 	0x54
#define sensor2_addr 	0x56

#define LONG_RANGE ((sensor_modes) 0 )
#define HIGH_SPEED ((sensor_modes) 1 )
#define HIGH_ACCURACY ((sensor_modes) 2 )
typedef uint8_t sensor_modes;

struct sensors{
	VL53L0X_RangingMeasurementData_t RangingData[sensor_qty];
	VL53L0X_Dev_t  vl53l0x_c[sensor_qty];
	VL53L0X_DEV Dev_array[sensor_qty];
	};


void Sensor_Init(struct sensors s , sensor_modes mode);
void Sensor_Config(VL53L0X_DEV dev,sensor_modes mode);
void Get_Data_Sensors(struct sensors s, uint32_t* data1, uint32_t* data2);

#endif /* INC_SENSORS_H_ */
