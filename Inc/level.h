#ifndef LEVEL_H
#define LEVEL_H

#include "stm32f405xx.h"

// Configuration defines
#define ADC_MIN 90   // ADC value when sensor is dry
#define ADC_MAX 2000   // ADC value when fully submerged

// Function prototypes
void water_level_init(void);
uint16_t water_level_read_raw(void);
float water_level_read_percent(void);
void delay_ms(uint32_t ms);

#endif // LEVEL_H
