#include "dht11.h"
#include "lcd.h"
#include "cmn.h"

DHT11_Data dht;
volatile uint8_t temperature_alert = 0;
volatile uint8_t humidity_alert = 0;

#define GPIO_PIN_2   (1 << 2)   // DHT11 on PA2
#define PC10_PIN     (1 << 10)  // Motor / Alert LED

// ---------------- DWT Delay ----------------
static void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

static void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

// ---------------- DHT11 Helpers ----------------
static void DHT11_Set_Pin_Output(void) {
    GPIOA->MODER &= ~(3 << (2 * 2));
    GPIOA->MODER |= (1 << (2 * 2));   // Output
    GPIOA->PUPDR &= ~(3 << (2 * 2));
}

static void DHT11_Set_Pin_Input(void) {
    GPIOA->MODER &= ~(3 << (2 * 2));  // Input
    GPIOA->PUPDR &= ~(3 << (2 * 2));
    GPIOA->PUPDR |= (1 << (2 * 2));   // Pull-up
}

static void DHT11_Start(void) {
    DHT11_Set_Pin_Output();
    GPIOA->ODR &= ~GPIO_PIN_2;
    delay_ms(20);
    GPIOA->ODR |= GPIO_PIN_2;
    delay_us(30);
    DHT11_Set_Pin_Input();
}

static uint8_t DHT11_Check_Response(void) {
    delay_us(40);
    if (!(GPIOA->IDR & GPIO_PIN_2)) {
        delay_us(80);
        if (GPIOA->IDR & GPIO_PIN_2) {
            delay_us(80);
            return 1;
        }
    }
    return 0;
}

static uint8_t DHT11_Read(void) {
    uint8_t i, data = 0;
    for (i = 0; i < 8; i++) {
        while (!(GPIOA->IDR & GPIO_PIN_2));
        delay_us(40);
        if (GPIOA->IDR & GPIO_PIN_2)
            data |= (1 << (7 - i));
        while (GPIOA->IDR & GPIO_PIN_2);
    }
    return data;
}

static uint8_t DHT11_GetData(DHT11_Data *data) {
    uint8_t Rh1, Rh2, Temp1, Temp2, checksum;
    DHT11_Start();
    if (DHT11_Check_Response()) {
        Rh1 = DHT11_Read();
        Rh2 = DHT11_Read();
        Temp1 = DHT11_Read();
        Temp2 = DHT11_Read();
        checksum = DHT11_Read();
        if (checksum == (Rh1 + Rh2 + Temp1 + Temp2)) {
            data->Humidity = Rh1;
            data->Temperature = Temp1;
            return 1;
        }
    }
    return 0;
}

// ---------------- Public Functions ----------------
void DHT11_Init(void) {
    // PA2 default high
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    GPIOA->ODR |= GPIO_PIN_2;

    // PC10 as motor control pin
    GPIOC->MODER &= ~(3 << (10 * 2));
    GPIOC->MODER |= (1 << (10 * 2));
}

void TIM4_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 7200 - 1;
    TIM4->ARR = 20000 - 1;  // ~2s interval
    TIM4->DIER |= TIM_DIER_UIE;
    TIM4->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM4_IRQn);
}

// ---------------- Timer4 Interrupt Handler ----------------
void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR &= ~TIM_SR_UIF;

        if (DHT11_GetData(&dht)) {
            temperature_alert = dht.Temperature;
            humidity_alert = dht.Humidity;

//            lprint(0x80, "T:");
//            lprint_num(0x83, temperature_alert);
//            lprint(0x86, "H:");
//            lprint_num(0x89, humidity_alert);

            // Motor control logic
            if (temperature_alert > 24 && humidity_alert > 80) {
                GPIOC->ODR |= PC10_PIN;
                //lprint(0xC0, "MOTOR ON...!");
                delay_ms(5000);
                GPIOC->ODR &= ~PC10_PIN;
               // lprint(0xC0, "MOTOR OFF   ");
            }
        }
    }
}
