#ifndef RFID_H_
#define RFID_H_

#include <stdint.h>

void RFID_Init(void);
uint8_t RFID_Scan(uint64_t* uid_val, char* uid_str);
void RFID_delay_ms(uint32_t ms);
extern uint8_t rc522_version;
extern uint8_t debug_status;
#endif /* RFID_H_ */
