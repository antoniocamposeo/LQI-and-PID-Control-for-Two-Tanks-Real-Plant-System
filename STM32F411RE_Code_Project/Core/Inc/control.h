/**
 * @file System.h
 *
 * @brief This header file defines the structures used for the control system of a pump and a tank.
 *
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "gpio.h"
#include <math.h>
#include <stdbool.h>

/**
 * @brief This structure defines the parameters of a pump.
 *
 * This structure contains the current and target power of the pump, the maximum and minimum power levels that the pump can operate at,
 * as well as the number of ticks for level and total ticks.
 */

volatile typedef struct Pump
{
	float PowerCurrent; /**< The current power of the pump. */
}Pump_t;

/**
 * @brief This structure defines the parameters of a tank.
 *
 * This structure contains the height, the current and target level of the tank.
 */
volatile typedef struct Tank
{
	float level_current; /**< The current level of the tank. */
	float level_target; /**< The target level of the tank. */
	float height; /**< Total height of the tank. */
	float sensor_height; /**< Height of the sensor respect to the base of the tank. */
}Tank_Param_t;

/**
 * @brief This structure defines the parameters of a PID controller.
 *
 * This structure contains the proportional, integral, and derivative gain constants of the PID controller, the sampling time,
 * the current and previous error and output, as well as the saturation flag.
 */
volatile typedef struct PID
{
	float Kp; /**< The proportional gain constant of the PID controller. */
	float Ki; /**< The integral gain constant of the PID controller. */
	float Kd; /**< The derivative gain constant of the PID controller. */
	float deltaT; /**< The sampling time of the PID controller. */
	float current_error; /**< The current error of the PID controller. */
	float current_output; /**< The current output of the PID controller. */
	float prev_error; /**< The previous error of the PID controller. */
	float prev_output; /**< The previous output of the PID controller. */
	float integral; /**< The integral term of the PID controller. */
	float derivative; /**< The derivative term of the PID controller. */
	bool saturation; /**< The saturation flag of the PID controller. */
}PID_Param_t;

/**
 * @brief This structure defines the parameters of an LQR controller.
 *
 * This structure contains the gain constants of the LQR controller, the sampling time,
 * the current and previous error and output, as well as the saturation flag.
 */
volatile typedef struct LQR
{
	float K1; /**< The first gain constant of the LQR controller. */
	float K2; /**< The second gain constant of the LQR controller. */
	float deltaT; /**< The sampling time of the LQR controller. */
	float current_error; /**< The current error of the LQR controller. */
	float current_output; /**< The current output of the LQR controller. */
	float prev_error; /**< The previous error of the LQR controller. */
	float prev_output; /**< The previous output of the LQR controller. */
	bool saturation; /**< The saturation flag of the LQR controller. */
}LQR_Param_t;

/**
 * @brief Struct defining the parameters for a LQI (Linear Quadratic Integral) controller
 *
 * This structure contains the gain constants of the LQI controller, the sampling time,
 * the accumulated integral value of the error, the current and previous error and output,
 * as well as the saturation flag.
 */
volatile typedef struct LQI_Param
{
    float K1;           //!< Coefficient for the proportional gain
    float K2;           //!< Coefficient for the integral gain
    float K3;           //!< Coefficient for the derivative gain

    float deltaT;       //!< Time step for the controller

    float current_error;    //!< Current error value
    float current_output;   //!< Current output value
    float prev_error;       //!< Previous error value
    float prev_output;      //!< Previous output value

    float integral;     //!< Accumulated integral value of the error
    bool saturation;    //!< Flag indicating if saturation occurred during the last execution
} LQI_Param_t;


/**
 * @brief Initializes the PID controller with default values.
 * @param PID Pointer to the PID controller parameters structure.
 */
void PID_Init(PID_Param_t* PID);

/**
 * @brief Initialize a Pump struct with default values.
 *
 * @param pump Pointer to the Pump struct to be initialized.
 */
void Pump_Init(Pump_t* pump);

/**
 * @brief Initialize a Tank_Param_t struct with default values.
 *
 * @param tank Pointer to the Tank_Param_t struct to be initialized.
 */
void Tank1_Init(Tank_Param_t* tank);
void Tank2_Init(Tank_Param_t* tank);
/**
 * @brief Initialize a LQR_Param_t struct with default values.
 *
 * @param lqr Pointer to the LQR_Param_t struct to be initialized.
 */
/**
 * @brief Initialize a LQI_Param_t struct with default values.
 *
 * @param lqi Pointer to the LQI_Param_t struct to be initialized.
 */


void LQI_Init(LQI_Param_t* lqi);
float PID_Control(PID_Param_t* PID, Tank_Param_t* Tank);
float LQR_Control(LQR_Param_t* LQR, Tank_Param_t* Tank1, Tank_Param_t* Tank2);
float LQI_Control(LQI_Param_t* LQI, Tank_Param_t* Tank1, Tank_Param_t* Tank2);
void actuation_pump(Pump_t* pump,float DC);
//
void MapLevel(uint32_t *data1,uint32_t *data2, Tank_Param_t* tank1,Tank_Param_t* tank2,float* lev1,float* lev2);

#endif /* SYSTEM_H_ */

