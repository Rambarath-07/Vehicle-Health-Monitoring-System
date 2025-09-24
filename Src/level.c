#include "level.h"

// Private variables
static uint16_t waterValue = 0;       // Raw ADC value
static float waterPercent = 0.0f;     // Calculated water percentage

/**
 * @brief Initialize water level sensor (ADC and GPIO)
 */
void water_level_init(void) {
    // 1. Enable GPIOC and ADC1 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // 2. Configure PC1 as analog input (ADC1_IN11)
    GPIOC->MODER |= (3UL << (1 * 2));  // Analog mode
    GPIOC->PUPDR &= ~(3UL << (1 * 2)); // No pull-up/down

    // 3. Configure ADC1
    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_ADON;
    ADC1->SMPR1 |= (3UL << 9);       // Sampling time for channel 11
    ADC1->SQR3 = 11;                 // First conversion = channel 11
}

/**
 * @brief Read raw ADC value from water level sensor
 * @return Raw ADC value (0-4095)
 */
uint16_t water_level_read_raw(void) {
    // Start ADC conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while(!(ADC1->SR & ADC_SR_EOC));

    // Read and return ADC value
    waterValue = ADC1->DR;
    return waterValue;
}

/**
 * @brief Read water level as percentage (0-100%)
 * @return Water level percentage (0.0 - 100.0)
 */
float water_level_read_percent(void) {
    // Get raw ADC value
    waterValue = water_level_read_raw();

    // Clamp to valid range
    if(waterValue < ADC_MIN) waterValue = ADC_MIN;
    if(waterValue > ADC_MAX) waterValue = ADC_MAX;

    // Convert to percentage
    waterPercent = ((float)(waterValue - ADC_MIN) / (ADC_MAX - ADC_MIN)) * 100.0f;

    // Clamp to 0-100%
    if(waterPercent < 0) waterPercent = 0;
    if(waterPercent > 100) waterPercent = 100;

    return waterPercent;
}


void delay_ms(uint32_t ms) {
    for(uint32_t i = 0; i < ms * 4000; i++) __NOP();
}
