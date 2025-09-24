#ifndef DHT11_H
#define DHT11_H

#include "stm32f405xx.h"
#include <stdint.h>

// DHT11 Data Structure
typedef struct {
    uint8_t Temperature;
    uint8_t Humidity;
} DHT11_Data;

// Global instance
extern DHT11_Data dht;
extern volatile uint8_t temperature_alert;
extern volatile uint8_t humidity_alert;

void DHT11_Init(void);
void TIM4_Config(void);   // Timer for periodic sampling

#endif
