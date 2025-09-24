#ifndef IR_H
#define IR_H

#include "stm32f405xx.h"

/* Pin definitions */
#define IR_SENSOR_PIN     2    // PC2 (Reconfigured from PA2)
#define BUZZER_PIN        9    // PC9

/* Extern variables */
// This flag is set by the timer ISR and read by the main loop.
extern volatile uint8_t object_detected;

/* Function prototypes */
void GPIO_Init(void);
void TIM2_Init(void);

#endif // IR_H
