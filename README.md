# IR_receiver-PanasonicTV

This library allows you to comfortably decode protocol used by Panasonic TV IR remote controls.

## Implementation
  1. Import header file "IR_PanasonicTV.h" to your project
  2. Add "libIR_receiver-PanasonicTV-t85.a" or "libIR_receiver-PanasonicTV-t88.a" library (located inside the "Release" folder) to your project
  3. Meet following instructions:

Timer/Counter and External interrupt is used to decode Panasonic IR protocol.

**Hardware support:**

ATtiny85: 8-bit Timer/Counter0, INT0

ATtiny88: 8-bit Timer/Counter0, INT0

CPU clock is 8 MHz.

## How to use

- Connect output pin of IR receiver to PB2->ATtiny85 / PD2->ATtiny88 (pin 7/4).

- Execute IR_init(uint16_t clearBufferTime) function to set registers, interrupts and clearBufferTime (after this time, durationBuffer will be cleared). It is useful, when some parts of running program cause delay. The clearBufferTime should be set the same as the duration of the delay. If nothing causes delay, just write 0.

- Repetition for certain commands can be disabled using IR_disableRepetition(uint8_t systemCode, uint8_t command). For each command one IR_disableRepetition(uint8_t systemCode, uint8_t command) execution is needed. So while holding a button, your instructions related to the command are executed just once (IR_available() returns false).

- Disabled repetition can be enabled using IR_enableRepetition(uint8_t systemCode, uint8_t command) again.

- Then just check IR_available() function in loop.

- If IR_available() returns true, IR signal has been checked succesfully and then IR data (address and command) are available.

- Access to IR data is through IR_data structure named IR (IR.systemCode for systemCode, IR.address for address, IR.command for command).

- Make sure to execute sei() function to enable global interrupts.
