#include "stm32f405xx.h"
#include "level.h"
#include "RFID.h"   // Your RFID driver
#include "IR.h"     // Your IR driver
#include "lcd.h"
#include "cmn.h"
#include "vibration.h" // Vibration Header
#include "dht11.h"

uint32_t SystemCoreClock = 72000000;

/* --- Authorized RFID UIDs --- */
#define AUTHORIZED_UID_1 564438966758ULL
#define AUTHORIZED_UID_2 162997186330ULL

// Global variables for debugging/monitoring
volatile uint16_t waterValue = 0;       // Raw ADC value
volatile float waterPercent = 0.0f;     // Calculated water percentage
volatile uint8_t unlocked = 0;          // System lock state
volatile uint8_t data_update_flag = 0;  // Flag to indicate new sensor data

/* --- Pin Definitions --- */
#define BUZZER_PIN    (1U<<9)  // PC9

/* LED Bar Graph Pin Definitions */
#define LED1_PIN      (1U<<15) // PA15 - 5-20%
#define LED2_PIN      (1U<<12) // PA12 - 20-40%
#define LED3_PIN      (1U<<11) // PA11 - 40-60%
#define LED4_PIN      (1U<<10) // PA10 - 60-70%
#define LED5_PIN      (1U<<9)  // PA9  - 70-90%
#define LED6_PIN      (1U<<8)  // PA8  - 90-100%

/* Global flag from IR.c, modified by an ISR */
extern volatile uint8_t object_detected;

/* --- Application-Specific Functions --- */
static inline void buzzer_on(void) {
	GPIOC->BSRR = BUZZER_PIN;
}
static inline void buzzer_off(void) {
	GPIOC->BSRR = (BUZZER_PIN << 16);
}

/* --- LED Control Functions --- */
static inline void led_on(uint32_t led_pin) {
	GPIOA->BSRR = led_pin;
}

static inline void led_off(uint32_t led_pin) {
	GPIOA->BSRR = (led_pin << 16);
}

static void all_leds_off(void) {
	led_off(LED1_PIN);
	led_off(LED2_PIN);
	led_off(LED3_PIN);
	led_off(LED4_PIN);
	led_off(LED5_PIN);
	led_off(LED6_PIN);
}

/* --- Water Level LED Bar Graph Control --- */
static void update_led_bar_graph(float water_percent) {
	// Turn off all LEDs first
	all_leds_off();

	// If water level is 0%, keep all LEDs off
	if (water_percent <= 0.0f) {
		return;
	}

	// Turn on LEDs based on water percentage ranges
	// Use >= for cumulative effect (fuel gauge style)
	if (water_percent >= 5.0f) {   // 5-20%: PA15
		led_on(LED1_PIN);
	}
	if (water_percent >= 20.0f) {  // 20-40%: PA12
		led_on(LED2_PIN);
	}
	if (water_percent >= 40.0f) {  // 40-60%: PA11
		led_on(LED3_PIN);
	}
	if (water_percent >= 60.0f) {  // 60-70%: PA10
		led_on(LED4_PIN);
	}
	if (water_percent >= 70.0f) {  // 70-90%: PA9
		led_on(LED5_PIN);
	}
	if (water_percent >= 90.0f) {  // 90-100%: PA8
		led_on(LED6_PIN);
	}

	// Debug: Force PA8 on when water_percent is exactly 100%
	if (water_percent >= 100.0f) {
		GPIOA->BSRR = LED6_PIN;  // Directly set PA8 high
	}
}

/* --- Timer 6 Interrupt Handler --- */
void TIM6_DAC_IRQHandler(void) {
	// Check if update interrupt flag is set
	if (TIM6->SR & TIM_SR_UIF) {
		// Clear the interrupt flag
		TIM6->SR &= ~TIM_SR_UIF;

		// Only read sensors when system is unlocked
		if (unlocked) {
			// Read water level sensor
			waterValue = water_level_read_raw();
			waterPercent = water_level_read_percent();

			// Set flag to indicate new data is available
			data_update_flag = 1;
		}
	}
}

/* --- Timer 6 Initialization (500ms interrupt) --- */
static void timer6_init(void) {
	// Enable Timer 6 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// Configure Timer 6
	// Assuming APB1 clock is 42MHz (typical for STM32F405)
	// For 500ms: ARR = (42000000 * 0.5) / (PSC + 1) - 1
	// Using PSC = 41999, ARR = 499 gives us exactly 500ms
	TIM6->PSC = 41999;    // Prescaler: 42MHz / 42000 = 1kHz
	TIM6->ARR = 499;      // Auto-reload: 1kHz / 500 = 2Hz (500ms period)

	// Enable update interrupt
	TIM6->DIER |= TIM_DIER_UIE;

	// Enable Timer 6 interrupt in NVIC
	NVIC_SetPriority(TIM6_DAC_IRQn, 3); // Set priority (0-15, lower is higher priority)
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	// Start the timer
	TIM6->CR1 |= TIM_CR1_CEN;
}

/* --- Hardware Initialization --- */
static void gpio_init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
			| RCC_AHB1ENR_GPIOCEN;

	// SPI1 pins (PA4-CS, PA5-SCK, PA6-MISO, PA7-MOSI) and RST (PB1) for RFID
	GPIOA->MODER &= ~((3U << 8) | (3U << 10) | (3U << 12) | (3U << 14));
	GPIOA->MODER |= (1U << 8) | (2U << 10) | (2U << 12) | (2U << 14); // PA4=Output, PA5,6,7=AF
	GPIOA->OSPEEDR |= (3U << 8) | (3U << 10) | (3U << 12) | (3U << 14);
	GPIOA->AFR[0] &= ~((0xFU << 20) | (0xFU << 24) | (0xFU << 28));
	GPIOA->AFR[0] |= (5U << 20) | (5U << 24) | (5U << 28); // AF5 for SPI1
	GPIOA->BSRR = (1U << 4); // CS high

	// LED Bar Graph pins (PA8, PA9, PA10, PA11, PA12, PA15) as outputs
	// Clear mode bits for all LED pins
	GPIOA->MODER &= ~((3U << 16) | (3U << 18) | (3U << 20) | (3U << 22)
			| (3U << 24) | (3U << 30));
	// Set as GPIO outputs (01)
	GPIOA->MODER |= (1U << 16) | (1U << 18) | (1U << 20) | (1U << 22)
			| (1U << 24) | (1U << 30);
	// Set high speed for all LED pins
	GPIOA->OSPEEDR |= (3U << 16) | (3U << 18) | (3U << 20) | (3U << 22)
			| (3U << 24) | (3U << 30);

	// Ensure PA8 and PA9 are not configured for alternate function (clear AFR bits)
	GPIOA->AFR[1] &= ~((0xFU << 0) | (0xFU << 4));  // Clear AF for PA8 and PA9

	// Set output type as push-pull (default, but ensure it's set)
	GPIOA->OTYPER &= ~(LED1_PIN | LED2_PIN | LED3_PIN | LED4_PIN | LED5_PIN
			| LED6_PIN);

	// Initialize all LEDs to OFF
	all_leds_off();

	// RST pin (PB1) as output
	GPIOB->MODER &= ~(3U << 2);
	GPIOB->MODER |= (1U << 2);
	GPIOB->OSPEEDR |= (3U << 2);
	GPIOB->BSRR = (1U << 1); // RST high

	// Buzzer pin (PC9) as output
	GPIOC->MODER &= ~(3U << 18);
	GPIOC->MODER |= (1U << 18);
	GPIOC->OSPEEDR |= (3U << 18);
	buzzer_off();
}

static void spi1_init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 = 0;
	SPI1->CR2 = 0;
	// Master mode, fPCLK/8 (check your APB2 clock), SSM & SSI enabled
	SPI1->CR1 |= SPI_CR1_MSTR | (2U << 3) | SPI_CR1_SSM | SPI_CR1_SSI;
	SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI
}

// ---------------- DWT Init ----------------
void DWT_Init(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// ---------------- Clock Config ----------------
void SystemClock_Config(void) {
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY))
		;
	RCC->PLLCFGR = (8 << 0) | (72 << 6) | RCC_PLLCFGR_PLLSRC_HSI;
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY))
		;
	FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
		;
}

/* --- Main Function --- */
int main(void) {
	uint64_t current_uid_val = 0;
	char current_uid_str[11] = { 0 };
	uint8_t card_present_previously = 0;

	// --- Hardware & Peripheral Initialization ---
	// Initialize water level sensor
	water_level_init();
	DHT11_Init();
	TIM4_Config();
	SystemClock_Config();
	DWT_Init();
	gpio_init();
	spi1_init();
	GPIO_Init(); // From IR.c
	TIM2_Init(); // From IR.c
	timer6_init(); // Initialize Timer 6 for sensor data acquisition
	Vibration_Init();
	RFID_delay_ms(100);
	LcdInit();
	RFID_Init();

	// System starts in the LOCKED state
	unlocked = 0;

	// --- Initial LCD Message ---
	lprint(0x80, "  System Locked  ");
	lprint(0xC0, " Tap Card to Use ");

	while (1) {
		delay_ms(50); // Reduced main loop delay since sensor reading is now timer-based

		// --- 1. Check for an RFID Card ---
		if (RFID_Scan(&current_uid_val, current_uid_str)) {
			// Process the card only ONCE per tap
			if (!card_present_previously) {
				card_present_previously = 1;

				// --- 2. Check if the Card is Authorized ---
				if (current_uid_val == AUTHORIZED_UID_1
						|| current_uid_val == AUTHORIZED_UID_2) {

					// --- 3. Toggle the System State (Lock/Unlock) ---
					unlocked = !unlocked;

					// --- 4. Update LCD and Reset State Based on New State ---
					if (unlocked) {
						lprint(0x80, " System Unlocked ");
						lprint(0xC0, "Sensor: .........");

						// IMPORTANT: Reset flag on unlock to ignore detections
						// that happened while the system was locked.
						object_detected = 0;
						data_update_flag = 0; // Reset data update flag
					} else {
						lprint(0x80, "  System Locked  ");
						lprint(0xC0, " Tap Card to Use ");
						// Turn off all LEDs when system is locked
						all_leds_off();
						data_update_flag = 0; // Reset data update flag
					}
				}
			}
		} else {
			card_present_previously = 0;
		}

		if (unlocked) {
			lprint(0xC7, "T:");
			lprint_num(0xC9, temperature_alert);
			lprint(0xCC, "H:");
			lprint_num(0xCE, humidity_alert);
		}

		// --- 5. Process Sensor Data and Perform Actions Based on Current State ---
		if (unlocked) {

			if (data_update_flag) {
				data_update_flag = 0; // Clear the flag

				// Update LED bar graph based on water percentage
				// Access volatile variables safely
				float current_water_percent = waterPercent;
				update_led_bar_graph(current_water_percent);
			}

			// Monitor IR sensor ONLY when unlocked
			if (object_detected) {
				lprint(0xC0, "IR:ON ");

				buzzer_on();
				RFID_delay_ms(300); // Alarm beep for 300ms
				buzzer_off();

				object_detected = 0; // Reset flag after processing the alert
			} else {
				lprint(0xC0, "IR:OFF");
			}
		} else {
			// When system is locked, turn off all LEDs
			all_leds_off();
		}
	}
	return 0;
}
