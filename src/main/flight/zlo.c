#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"
#include "zlo.h"
#include "flight/imu.h"
#include "rx/rx.h"

#define AUTO_SWITCH_ON (2)
#define AUTO_SWITCH_OFF (3)

#if !defined(SITL)
void sitl_printf(const char * pStr, ...) {
    UNUSED(pStr);
}
#else
extern void sitl_printf(const char * pStr, ...);
#endif

bool zloActivated = false;
bool zloActivationRequested = false;
bool zloDeactivationRequested = false;

PIDController yawPid = {.Kp=25.0f, .Ki=10.0f, .Kd=2.0f, .limMax=500.0f, .limMin=-500.0f, .limMaxInt=400.0f, .limMinInt=-400.0f},
    pitchPid={.Kp=20.0f, .Ki=120.0f, .Kd=5.0f, .limMax=500.0f, .limMin=-500.0f, .limMaxInt=400.0f, .limMinInt=-400.0f},
    throttlePid={.Kp=20.0f, .Ki=120.0f, .Kd=30.0f, .limMax=500.0f, .limMin=-500.0f, .limMaxInt=400.0f, .limMinInt=-400.0f},
    rollPid={.Kp=20.0f, .Ki=100.0f, .Kd=10.0f, .limMax=500.0f, .limMin=-500.0f, .limMaxInt=400.0f, .limMinInt=-400.0f};

uint16_t ch[4]; // AETR1234...

bool zloInit(void) {
    PIDController_Init(&yawPid);
    PIDController_Init(&pitchPid);
    PIDController_Init(&throttlePid);
    PIDController_Init(&rollPid);

    return true;
}

// from msp.h
// set up channels
void rxMspFrameReceive(const uint16_t *frame, int channelCount);

float _attitude[3] = {0.0f, 0.0f, 0.0f}; // ypr

timeUs_t prevTime = 0;

// UPDATE cycle
void zloUpdate(timeUs_t currentTimeUs) {

    uint16_t autoValueOn = (uint16_t)rxRuntimeState.rcReadRawFn(&rxRuntimeState, (AUTO_SWITCH_ON-1)+4);
    uint16_t autoValueOff = (uint16_t)rxRuntimeState.rcReadRawFn(&rxRuntimeState, (AUTO_SWITCH_OFF-1)+4);

    zloActivationRequested = (autoValueOn > 1500) && ! (zloActivated);
    zloDeactivationRequested = (autoValueOff > 1500) && zloActivated && !zloActivationRequested;

    if(zloActivationRequested && !zloActivated) {
        zloActivated = true;

        PIDController_Init(&yawPid);
        PIDController_Init(&pitchPid);
        PIDController_Init(&throttlePid);
        PIDController_Init(&rollPid);
        _attitude[2] = 0.0f;
    } else if (zloDeactivationRequested && !zloActivationRequested && zloActivated) {
        zloActivated = false;
    }

    if (zloActivated && prevTime != 0) {
        // 0. set target roll = 0; DONE earlier

        // 1. get attitude
        // 2. update pids
        float sampleTime = ((float)(currentTimeUs-prevTime)) / 1000000.0f;
        PIDController_Update(&rollPid, _attitude[2], attitude.values.roll/10.0f, sampleTime); // attitude[2] is always zero
        PIDController_Update(&pitchPid, _attitude[1], attitude.values.pitch/10.0f, sampleTime);
        PIDController_Update(&yawPid, _attitude[0], attitude.values.yaw/10.0f, sampleTime);

        // 3. update rxMsp AETR channels from yaw, pitch, roll PID
        uint16_t frame[4] = {
            (uint16_t)(ch[0] + yawPid.out),
            (uint16_t)(ch[1] + pitchPid.out),
            (uint16_t)(ch[2] + throttlePid.out), // .out is always zero
            (uint16_t)(ch[3] + rollPid.out)
            };
        rxMspFrameReceive(frame, 4);
    } else { // no autonomous
        // attitute
        _attitude[0] = attitude.values.yaw / 10.0f;
        _attitude[1] = attitude.values.pitch / 10.0f;
        _attitude[2] = attitude.values.roll / 10.0f;

        // save AETR channels to yaw, roll, pitch, throttle
        ch[0] = (uint16_t)rxRuntimeState.rcReadRawFn(&rxRuntimeState, 0); // A
        ch[1] = (uint16_t)rxRuntimeState.rcReadRawFn(&rxRuntimeState, 1); // E
        ch[2] = (uint16_t)rxRuntimeState.rcReadRawFn(&rxRuntimeState, 2); // T
        ch[3] = (uint16_t)rxRuntimeState.rcReadRawFn(&rxRuntimeState, 3); // R
        uint16_t frame[4] = {
            ch[0],
            ch[1],
            ch[2],
            ch[3]
            };
        rxMspFrameReceive(frame, 4);
    }

    prevTime = currentTimeUs;
    sitl_printf("Activated: %d; ", zloActivated);
    sitl_printf("YPR: %d, %d, %d; ***** A, E, T, R: %d, %d, %d, %d\n\r",
        (uint16_t)(attitude.values.yaw/10),
        (uint16_t)(attitude.values.pitch/10),
        (uint16_t)(attitude.values.roll/10),
        ch[0] + (uint16_t)yawPid.out, ch[1] + (uint16_t)pitchPid.out,
            ch[2] + (uint16_t)throttlePid.out, ch[3] + (uint16_t)rollPid.out
        );
}

bool overrideChannel(uint8_t chan) {
    return (/*chan >= 0 &&*/ chan <= 3);
}

// ----  PID ------

void PIDController_Init(PIDController *pid) {
    // Clear controller variables
    pid->integrator = 0.0f;
    pid->prevError  = 0.0f;
    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
}

/* Derivative low-pass filter time constant */
float TAU = .001f;

float PIDController_Update(PIDController *pid,
    /* target */ float setpoint,
    /* actual */ float measurement,
    /* Sample time (in seconds) */ float T) {

    // Error signal
    float error = setpoint - measurement;

    // Proportional
    float proportional = pid->Kp * error;

    // Integral
    pid->integrator = pid->integrator + pid->Ki * T * (error + pid->prevError);

    // Anti-wind-up via integrator clamping
    if (pid->integrator > pid->limMaxInt) {
        pid->integrator = pid->limMaxInt;
    } else if (pid->integrator < pid->limMinInt) {
        pid->integrator = pid->limMinInt;
    }

    // Derivative (band-limited differentiator)
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
        // Note: derivative on measurement, therefore minus sign in front of equation!
        + (2.0f * TAU - T) * pid->differentiator)
        / (2.0f * TAU + T);

    // Compute output and apply limits
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

    // Store error and measurement for later use
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    // Return controller output
    return pid->out;
}
