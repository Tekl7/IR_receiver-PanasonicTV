/*
 * IR_PanasonicTV.h
 *
 * Created: 13. 6. 2019 12:18:45
 *  Author: Vojta
 *
 * This project uses Timer/Counter and External interrupt to decode IR Panasonic TV protocol.
 *
 * Hardware support:
 * ATtiny85: 8-bit Timer/Counter0, INT0
 * ATtiny88: 8-bit Timer/Counter0, INT0
 * CPU clock is 8 MHz.
 *
 * Connect output pin of IR receiver to PB2/PD2 (pin 7/4).
 * Execute IR_init(uint16_t clearBufferTime) function to set registers, interrupts and clearBufferTime (after this time, durationBuffer will be cleared).
 * It is useful, when some parts of running program cause delay. The clearBufferTime should be set the same as the duration of the delay.
 * Repetition for certain buttons can be disabled using IR_disableRepetition(uint8_t systemCode, uint8_t command).
 * For each button one IR_disableRepetition(uint8_t systemCode, uint8_t command) execution is needed.
 * So while holding a button, your instructions related to the systemCode and command are executed just once (IR_available() returns false).
 * Disabled repetition can be enabled using IR_enableRepetition(uint8_t systemCode, uint8_t command) again.
 * Then just check IR_available() function in loop.
 * If IR_available() returns true, IR signal has been checked succesfully and then IR data (systemCode, address and command) are available.
 * Access to IR data is through IR_data structure named IR (IR.systemCode for systemCode, IR.address for address, IR.command for command).
 * Make sure to execute sei() function to enable global interrupts.
 */ 


#ifndef IR_PANASONICTV_H_
#define IR_PANASONICTV_H_

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <stdbool.h>

// Structure for systemCode, address and command
typedef struct
{
	uint8_t systemCode;
	uint8_t address;
	uint8_t command;
} IR_data;

extern IR_data IR;

void IR_init(uint16_t clearBufferTime);
void IR_disableRepetition(uint8_t systemCode, uint8_t command);
void IR_enableRepetition(uint8_t systemCode, uint8_t command);
bool IR_available();

#endif /* IR_PANASONICTV_H_ */