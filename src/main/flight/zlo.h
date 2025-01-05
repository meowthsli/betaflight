#pragma once

#include <stdint.h>
#include "common/time.h"

extern bool zloActivated;
extern bool zloActivationRequested;
extern bool zloDeactivationRequested;

bool zloInit(void);
void zloUpdate(timeUs_t currentTimeUs);
bool overrideChannel(uint8_t chan);

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Output limits */
	float limMin;
	float limMax;

	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;
} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement, float T);

