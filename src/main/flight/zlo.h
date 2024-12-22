#pragma once

#include <stdint.h>
#include "common/time.h"

extern bool zloActivated;
extern bool zloActivationRequested;
extern bool zloDeactivationRequested;

bool zloInit(void);
void zloUpdate(timeUs_t currentTimeUs);
bool overrideChannel(uint8_t chan);
