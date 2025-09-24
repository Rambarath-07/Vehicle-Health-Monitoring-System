#ifndef VIBRATION_H
#define VIBRATION_H

#include "stm32f405xx.h"
#include <stdint.h>

extern volatile uint32_t vibration_count;

void Vibration_Init(void);

#endif
