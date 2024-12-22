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

bool zloInit(void) {
    return true;
}

void zloUpdate(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    uint16_t autoValueOn = 0;
    uint16_t autoValueOff = 0;
    if(0) { // rxRuntimeState.channelData != 0) {
        autoValueOn = rxRuntimeState.channelData[(AUTO_SWITCH_ON-1)+4];
        autoValueOff = rxRuntimeState.channelData[(AUTO_SWITCH_OFF-1)+4];
    } else {
        autoValueOn = (uint16_t)rxRuntimeState.rcReadRawFn(&rxRuntimeState, (AUTO_SWITCH_ON-1)+4);
        autoValueOff = (uint16_t)rxRuntimeState.rcReadRawFn(&rxRuntimeState, (AUTO_SWITCH_OFF-1)+4);
    }
    zloActivationRequested = (autoValueOn > 1500) && ! (zloActivated);
    zloDeactivationRequested = (autoValueOff > 1500) && zloActivated && !zloActivationRequested;

    sitl_printf("ZloRequestedOn: %d; ZloRequestedOff: %d; Activated: %d; ",
        zloActivationRequested, zloDeactivationRequested, zloActivated);
}

bool overrideChannel(uint8_t chan) {
    return (/*chan >= 0 &&*/ chan <= 3);
}