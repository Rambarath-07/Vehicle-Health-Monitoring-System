#include "IR.h"

// The main loop will use this flag to decide when to sound the buzzer.
volatile uint8_t object_detected = 0;

/*
 * NOTE: The 'buzzer_counter' global variable has been removed, as the main loop
 * is now responsible for all buzzer timing and control.
 */

/* === GPIO Configuration === */
void GPIO_Init(void)
{
    /* Enable GPIOC clock (GPIOA is no longer needed for the IR sensor) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    /* PC2 as input (IR sensor) */
    // Set PC2 to input mode
    GPIOC->MODER &= ~(0x3U << (IR_SENSOR_PIN * 2));

    /*
     * Buzzer pin (PC9) configuration is already handled in the gpio_init()
     * function within main.c. Defining it here is redundant but harmless
     * as long as the settings are the same. For clarity, it's best to
     * initialize each peripheral's pins only once in a central location.
     */
    GPIOC->MODER &= ~(0x3U << (BUZZER_PIN * 2));
    GPIOC->MODER |=  (0x1U << (BUZZER_PIN * 2));      // output mode
}

/* === Timer2 Configuration === */
void TIM2_Init(void)
{
    /* Enable TIM2 clock */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /*
     * Configure timer for a 10 ms interrupt interval.
     * System Clock: 84 MHz (Assumed)
     * Prescaler: 8399 -> Timer Clock = 84 MHz / (8399 + 1) = 10 kHz
     * Auto-Reload Register: 99 -> Interrupt Freq = 10 kHz / (99 + 1) = 100 Hz
     * Interrupt Period = 1 / 100 Hz = 10 ms
     */
    TIM2->PSC = 8399;
    TIM2->ARR = 99;

    /* Enable Update Interrupt */
    TIM2->DIER |= TIM_DIER_UIE;

    /* Enable TIM2 Counter */
    TIM2->CR1 |= TIM_CR1_CEN;

    /* Enable TIM2 IRQ in the NVIC */
    NVIC_EnableIRQ(TIM2_IRQn);
}

/* === Timer2 ISR: Detects Object and Sets a Flag === */
void TIM2_IRQHandler(void)
{
    // Check if the update interrupt flag is set
    if (TIM2->SR & TIM_SR_UIF)
    {
        // Clear the update interrupt flag to prevent re-entry
        TIM2->SR &= ~TIM_SR_UIF;

        // Check the state of the IR sensor pin PC2
        if ((GPIOC->IDR & (1 << IR_SENSOR_PIN)) == 0)
        {
            // Set the global flag. The main loop will see this and take action.
            object_detected = 1;
        }
    }
}
