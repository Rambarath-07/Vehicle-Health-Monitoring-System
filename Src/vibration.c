#include "vibration.h"

#include "lcd.h"

volatile uint32_t vibration_count = 0;

volatile uint8_t beep_5_done = 0;

volatile uint8_t beep_10_done = 0;

static void delay_ms(uint32_t ms) {

    extern uint32_t SystemCoreClock;

    uint32_t ticks = (SystemCoreClock / 1000) * ms;

    uint32_t start = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < ticks);

}

void Vibration_Init(void) {

    // PA3 as input (vibration sensor)

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(3 << (3 * 2));

    GPIOA->PUPDR &= ~(3 << (3 * 2));

    GPIOA->PUPDR |= (2 << (3 * 2));  // Pull-down

    // PC9 buzzer

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    GPIOC->MODER &= ~(3 << (9 * 2));

    GPIOC->MODER |= (1 << (9 * 2));

    // EXTI3 interrupt

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SYSCFG->EXTICR[0] &= ~(0xF << 12);

    SYSCFG->EXTICR[0] |= (0 << 12);  // PA3

    EXTI->IMR |= (1 << 3);

    EXTI->RTSR |= (1 << 3);

    NVIC_EnableIRQ(EXTI3_IRQn);

}

void EXTI3_IRQHandler(void) {

    if (EXTI->PR & (1 << 3)) {

        vibration_count++;

        delay_ms(50);

        //lprint(0xC0, "V:");
       // lprint_num(0xC2, vibration_count);
        if (vibration_count >= 5 && !beep_5_done) {
            beep_5_done = 1;

            GPIOC->BSRR = GPIO_BSRR_BS9;

            delay_ms(500);

            GPIOC->BSRR = GPIO_BSRR_BR9;

        }

        if (vibration_count >= 10 && !beep_10_done) {

            beep_10_done = 1;

            GPIOC->BSRR = GPIO_BSRR_BS9;

            delay_ms(1000);

            GPIOC->BSRR = GPIO_BSRR_BR9;

        }

        if (vibration_count >= 15) {

            vibration_count = 0;

            beep_5_done = 0;

            beep_10_done = 0;

            //lprint(0xC0, "V:0  ");

        }

        EXTI->PR = (1 << 3); // clear interrupt

    }

}




