#include "RFID.h"
#include "stm32f405xx.h"
#include <string.h>
#include <stdio.h>

/* --- Pin Definitions --- */
#define RC522_CS_PIN  (1U<<4)  // PA4
#define RC522_RST_PIN (1U<<1)  // PB1

/* --- MFRC522 Register Definitions --- */
#define CommandReg      0x01
#define ComIEnReg       0x02
#define ComIrqReg       0x04
#define ErrorReg        0x06
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0A
#define ControlReg      0x0C
#define BitFramingReg   0x0D
#define ModeReg         0x11
#define TxControlReg    0x14
#define TxAutoReg       0x15
#define TModeReg        0x2A
#define TPrescalerReg   0x2B
#define TReloadRegH     0x2C
#define TReloadRegL     0x2D
#define VersionReg      0x37

/* --- MFRC522 Commands --- */
#define PCD_IDLE        0x00
#define PCD_TRANSCEIVE  0x0C
#define PCD_SOFTRESET   0x0F

/* --- MIFARE Commands --- */
#define PICC_REQIDL     0x26
#define PICC_ANTICOLL   0x93

/* --- Global variables (internal to this file) --- */
uint8_t  scannedUID_internal[10] = {0};

/* --- Global variables (for debugging via RFID.h) --- */
uint8_t rc522_version = 0;
uint8_t debug_status = 0;

/* --- Static (Private) Function Prototypes --- */
static void rc522_init(void);
static uint8_t rc522_scan_internal(uint8_t *id);
// ... and all other helper functions

/* --- Delay function --- */
void RFID_delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 8000; i++) {
        __asm__("nop");
    }
}

/* --- Low-Level Hardware and Register Functions (Static) --- */
static inline void rc522_cs_low(void) { GPIOA->BSRR = (RC522_CS_PIN << 16); }
static inline void rc522_cs_high(void) { GPIOA->BSRR = RC522_CS_PIN; }

static uint8_t spi1_transfer(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));
    *(volatile uint8_t*)&SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return *(volatile uint8_t*)&SPI1->DR;
}

static void rc522_write_reg(uint8_t reg, uint8_t val) {
    rc522_cs_low();
    spi1_transfer((reg << 1) & 0x7E);
    spi1_transfer(val);
    rc522_cs_high();
}

static uint8_t rc522_read_reg(uint8_t reg) {
    uint8_t val;
    rc522_cs_low();
    spi1_transfer(((reg << 1) & 0x7E) | 0x80);
    val = spi1_transfer(0x00);
    rc522_cs_high();
    return val;
}

static void rc522_set_bit_mask(uint8_t reg, uint8_t mask) {
    rc522_write_reg(reg, rc522_read_reg(reg) | mask);
}

static void rc522_clear_bit_mask(uint8_t reg, uint8_t mask) {
    rc522_write_reg(reg, rc522_read_reg(reg) & (~mask));
}

static void rc522_reset(void) {
    GPIOB->BSRR = (RC522_RST_PIN << 16);
    RFID_delay_ms(10);
    GPIOB->BSRR = RC522_RST_PIN;
    RFID_delay_ms(50);
}

static void rc522_init(void) {
    debug_status = 0;
    rc522_reset();
    rc522_write_reg(CommandReg, PCD_SOFTRESET);
    RFID_delay_ms(50);
    while (rc522_read_reg(CommandReg) & (1 << 4));
    rc522_version = rc522_read_reg(VersionReg);
    rc522_write_reg(TModeReg, 0x8D);
    rc522_write_reg(TPrescalerReg, 0x3E);
    rc522_write_reg(TReloadRegL, 0x1E);
    rc522_write_reg(TReloadRegH, 0x00);
    rc522_write_reg(TxAutoReg, 0x40);
    rc522_write_reg(ModeReg, 0x3D);
    rc522_set_bit_mask(TxControlReg, 0x03);
    debug_status = 1;
}

static uint8_t rc522_to_card(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen) {
    uint8_t status = 0, irqEn = 0x00, waitIRq = 0x00, lastBits, n;
    uint16_t i;
    if (command == PCD_TRANSCEIVE) {
        irqEn = 0x77;
        waitIRq = 0x30;
    }
    rc522_write_reg(ComIEnReg, irqEn | 0x80);
    rc522_write_reg(ComIrqReg, 0x7F);
    rc522_set_bit_mask(FIFOLevelReg, 0x80);
    rc522_write_reg(CommandReg, PCD_IDLE);
    for (i = 0; i < sendLen; i++) rc522_write_reg(FIFODataReg, sendData[i]);
    rc522_write_reg(CommandReg, command);
    if (command == PCD_TRANSCEIVE) rc522_set_bit_mask(BitFramingReg, 0x80);
    i = 2000;
    do {
        n = rc522_read_reg(ComIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));
    rc522_clear_bit_mask(BitFramingReg, 0x80);
    if (i != 0 && !(rc522_read_reg(ErrorReg) & 0x1B)) {
        status = 1;
        if (n & irqEn & 0x01) status = 0;
        if (command == PCD_TRANSCEIVE) {
            n = rc522_read_reg(FIFOLevelReg);
            lastBits = rc522_read_reg(ControlReg) & 0x07;
            *backLen = (lastBits) ? (n - 1) * 8 + lastBits : n * 8;
            if (n == 0) n = 1;
            if (n > 16) n = 16;
            for (i = 0; i < n; i++) backData[i] = rc522_read_reg(FIFODataReg);
        }
    }
    return status;
}

static uint8_t rc522_request(uint8_t reqMode, uint8_t *TagType) {
    uint16_t backBits;
    rc522_write_reg(BitFramingReg, 0x07);
    TagType[0] = reqMode;
    if (rc522_to_card(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits) != 1 || backBits != 0x10) return 0;
    return 1;
}

static uint8_t rc522_anticoll(uint8_t *serNum) {
    uint16_t unLen;
    uint8_t serNumCheck = 0;
    rc522_write_reg(BitFramingReg, 0x00);
    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    if (rc522_to_card(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen) == 1) {
        for (uint8_t i = 0; i < 4; i++) serNumCheck ^= serNum[i];
        if (serNumCheck != serNum[4]) return 0;
        return 1;
    }
    return 0;
}

static uint8_t rc522_scan_internal(uint8_t *id) {
    debug_status = 2;
    if (rc522_request(PICC_REQIDL, id) == 1 && rc522_anticoll(id) == 1) {
        debug_status = 3;
        return 1;
    }
    return 0;
}


/* --- Public Function Implementations --- */

void RFID_Init(void) {
    rc522_init();
}

uint8_t RFID_Scan(uint64_t* uid_val, char* uid_str) {
    uint8_t card_id[5];
    if (rc522_scan_internal(card_id)) {
        // Copy to internal buffer for safety
        memcpy(scannedUID_internal, card_id, 5);

        // Format UID into a hex string for the user
        sprintf(uid_str, "%02X%02X%02X%02X%02X",
                scannedUID_internal[0], scannedUID_internal[1],
                scannedUID_internal[2], scannedUID_internal[3], scannedUID_internal[4]);

        // Convert UID to a single numerical value for the user
        *uid_val = 0;
        *uid_val |= (uint64_t)scannedUID_internal[0] << 32;
        *uid_val |= (uint64_t)scannedUID_internal[1] << 24;
        *uid_val |= (uint64_t)scannedUID_internal[2] << 16;
        *uid_val |= (uint64_t)scannedUID_internal[3] << 8;
        *uid_val |= (uint64_t)scannedUID_internal[4];

        return 1; // Success
    }
    return 0; // Failure
}
