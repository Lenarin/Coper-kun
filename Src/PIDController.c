#include <PIDController.h>


float defaultErrorFunc(float a, float b) {
	return b - a;
}

/**
 * @brief Init PID configs with params
 *
 * @param props     : pointer to a settings structure
 * @param newKp     : value of Kp
 * @param newKi     : value of Ki
 * @param newKd     : value of Kd
 * @param newMax    : maximum output value
 * @param newMin    : minimum output values
 *
 * @retval None
 */
void InitPID(PIDProp *props, float newKp, float newKi, float newKd, float newMax, float newMin) {
	props->Kp = newKp;
	props->Ki = newKi;
	props->Kd = newKd;
	props->integralSum = 0;
	props->maxOutput = newMax;
	props->minOutput = newMin;
	props->pointValue = 0;
	props->lastInput = 0;
	props->lastOutput = 0;
	props->lastError = 0;
	props->enabled = enabledON;
	props->mode = 0;
	props->lasttime = HAL_GetTick();
	props->lastdelay = 1;
	props->errorFunc = defaultErrorFunc;
}

/**
 * @brief set new maximum output value
 *
 * @param props 	 :	pointer to a settings structure
 * @param value	 	 : 	new maximum output value
 *
 * @retval None
 */
void setMax(PIDProp *props, uint8_t value) {
	props->maxOutput = value;
}

/**
 * @brief set new minimum output value
 *
 * @param props 	 :	pointer to a settings structure
 * @param value		 : 	new minimum output value
 *
 * @retval None
 */
void setMin(PIDProp *props, uint8_t value) {
	props->minOutput = value;
}

/**
 * @brief set pointer value for the controller
 *
 * @param props		:	pointer to a settings structure
 * @param value		: 	new point value
 *
 * @retval None
 */
void setPoint(PIDProp *props, float value) {
	props->pointValue = value;
}

/**
 * @brief Try to calc new value if time has come
 *
 * @param props		:	pointer to a settings structure
 * @param value		:	curret value of controlled model
 * @param dt		: 	elapsed time interval
 *
 * @retval Manipulating output value
 */
float calc(PIDProp *props, float inputValue, float dt) {
	//if (!props->enabled) return props->lastOutput;
	//if (HAL_GetTick() - props->lasttime < UPTIME) return props->lastOutput;

	//float error = props->errorFunc(props->pointValue, inputValue);
	float error = inputValue - props->pointValue;
	float output = 0;

	props->integralSum += error * dt * props->Ki;
	//if (props->mode == 1) props->integralSum -= props->Kp * error;
	if (props->integralSum > props->maxOutput) props->integralSum = props->maxOutput;
	if (props->integralSum < props->minOutput) props->integralSum = props->minOutput;

	// Episilon dropout
	//if (abs(error) < 0.5f) props->integralSum = 0;

	output = error * props->Kp + props->integralSum  + (error - props->lastError) * props->Kd / dt;
	//if (props->mode == 1) output = 0;

	if (output > props->maxOutput) output = props->maxOutput;
	if (output < props->minOutput) output = props->minOutput;

	props->lastOutput = output;
	props->lastError = error;

	return output;
}

/**
 * @brief Set kofs of the controller with time estimate
 *
 * @param props		: pointer to a settings structure
 * @param newKp		: new Kp koef value
 * @param newKi		: new Ki koef value
 * @param newKd		: new Kd koef value
 *
 * @retval None
 */
void setKof(PIDProp *props, float newKp, float newKi, float newKd) {
	float time = (float)props->lastdelay / 1000;
	props->Kp = newKp;
	props->Ki = newKi * time;
	props->Kd = newKd / time;
}

/**
 * @brief Reset controller values to prevent bump start
 *
 * @param props		:	pointer to a setting structure
 *
 * @retval None
 */
void resetController(PIDProp *prop) {
	prop->integralSum = 0;
	prop->lasttime = 0;
	prop->lastOutput = 0;
	prop->lastInput = 0;
	prop->pointValue = 0;
	prop->lasttime = HAL_GetTick();
}

void dropISum(PIDProp *prop) {
	prop->integralSum = 0;
}


void setErrorFunc(PIDProp *prop, float (*fc) (float, float)) {
	prop->errorFunc = fc;
}
