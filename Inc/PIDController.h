
#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include "stm32f4xx_hal.h"

#define UPTIME 50
#define enabledON 1
#define enabledOFF 0

typedef struct PIDProp{
	float Ki, Kp, Kd;

	float integralSum;
	int32_t maxOutput, minOutput;
	float pointValue;
	float lastInput, lastOutput;
	uint32_t lasttime;
	uint32_t lastdelay;
	uint8_t enabled;
	uint8_t mode;
	float (*errorFunc) (float, float);
} PIDProp;

void setMax(PIDProp *props, uint8_t value);
void setMin(PIDProp *props, uint8_t value);
void setPoint(PIDProp *props, float value);
float calc(PIDProp *props, float inputValue);
void setKof(PIDProp *props, float newKp, float newKi, float newKd);
void InitPID(PIDProp *props, float newKp, float newKi, float newKd, float newMax, float newMin);
void resetController(PIDProp *prop);
void setErrorFunc(PIDProp *prop, float (*fc) (float, float));


#endif /* PIDCONTROLLER_H_ */
