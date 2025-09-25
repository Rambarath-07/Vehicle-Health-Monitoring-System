#include "stm32f405xx.h"
#include "level.h"
#include "RFID.h"   // Your RFID driver
#include "IR.h"     // Your IR driver
#include "lcd.h"
#include "cmn.h"

/* --- Authorized RFID UIDs --- */
#define AUTHORIZED_UID_1 564438966758ULL
#define AUTHORIZED_UID_2 162997186330ULL

// Global variables for debugging/monitoring
uint16_t waterValue = 0;       // Raw ADC value
float waterPercent = 0.0f;     // Calculated water percentage

/* --- Pin Definitions --- */
#define BUZZER_PIN    (1U<<9)  // PC9

/* Global flag from IR.c, modified by an ISR */
extern volatile uint8_t object_detected;

/* --- Application-Specific Functions --- */
static inline void buzzer_on(void) {
	GPIOC->BSRR = BUZZER_PIN;
}
static inline void buzzer_off(void) {
	GPIOC->BSRR = (BUZZER_PIN << 16);
}

/* --- Hardware Initialization (gpio_init, spi1_init) --- */
// (Your existing init functions go here, no changes needed)
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

/* --- Main Function --- */
int main(void) {
	uint64_t current_uid_val = 0;
	char current_uid_str[11] = { 0 };
	uint8_t card_present_previously = 0;
	uint8_t unlocked = 0; // System starts in the LOCKED state

	// --- Hardware & Peripheral Initialization ---
	// Initialize water level sensor
	water_level_init();
	gpio_init();
	spi1_init();
	GPIO_Init(); // From IR.c
	TIM2_Init(); // From IR.c
	RFID_delay_ms(100);
	LcdInit();
	RFID_Init();

	// --- Initial LCD Message ---
	lprint(0x80, "  System Locked  ");
	lprint(0xC0, " Tap Card to Use ");

	while (1) {

		waterValue = water_level_read_raw();
		waterPercent = water_level_read_percent();

		delay_ms(200);

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
					} else {
						lprint(0x80, "  System Locked  ");
						lprint(0xC0, " Tap Card to Use ");
					}
				}
			}
		} else {
			card_present_previously = 0;
		}

		// --- 5. Perform Actions Based on the Current State ---
		if (unlocked) {
			// Monitor sensor ONLY when unlocked
			if (object_detected) {
				lprint(0xC0, "Sensor: Detected ");

				buzzer_on();
				RFID_delay_ms(300); // Alarm beep for 300ms
				buzzer_off();

				object_detected = 0; // Reset flag after processing the alert
			} else {
				lprint(0xC0, "Sensor: Not Det. ");
			}
		}

		RFID_delay_ms(50); // Loop delay
	}
	return 0;
}

