/*
 * IR_receiver-PanasonicTV.c
 *
 * Created: 13. 6. 2019 12:17:38
 * Author : Vojta
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <stdlib.h>

#include "IR_PanasonicTV.h"

// Choose a number and assign it to DEVICE macro to select the demanded device.
// ATtiny85 -> 0
// ATtiny88 -> 1
#define DEVICE 0

// Times in µs
#define LEADING_PULSE 3380
#define INITIAL_SPACE 1690
#define FINAL_PULSE 420
#define LOGIC_SHORT 420
#define LOGIC_LONG 1270

// One tick duration in µs
#define TICK 128
#define NUM_OF_TICKS 2

// States
#define SLEEP_STATE 0
#define LEADING_PULSE_STATE 1
#define LEADING_SPACE_STATE 2
#define NON_SAVING_STATE 3
#define SYSTEM_CODE_STATE 4
#define ADDRESS_STATE 5
#define COMMAND_STATE 6
#define FINAL_PULSE_STATE 7
#define WAIT_STATE 8

// Number of OVFs to SLEEP_STATE or clear buffer
#define NUM_OF_OVFS 3
#define NUM_OF_CLEAR_BUFFER_OVFS(clearBufferTime) (clearBufferTime + 16) / (1UL * 1000UL * 256UL * 1024UL / F_CPU)

// Number of elements in the durationBuffer
#define BUFFER_SIZE 36

IR_data IR;
static uint8_t numOfButtons = 0;
static uint8_t *buttonsPtr = NULL;
static uint16_t timeRange = TICK * NUM_OF_TICKS;
static uint16_t clearBTime = 0;
static volatile bool durationBuffer[BUFFER_SIZE];	// LOGIC_SHORT -> true, LOGIC_LONG -> false
static volatile uint8_t bufferIndex = 0, nonSavingCounter = 0;
static volatile bool bufferReady = false;
static volatile bool clearBuffer = false;
static volatile bool level = true;					// Pulse -> level = true, space -> level = false
static volatile uint8_t state = SLEEP_STATE;
static volatile uint16_t ovfCounter = 1;
static volatile bool available = false;

static bool isRepDisabled(uint8_t systemCode, uint8_t command);
static void writeBit(uint8_t index, uint8_t offset, uint8_t *data);
static void setSleepState();

void IR_init(uint16_t clearBufferTime)
{
	#if DEVICE == 0
	// Normal mode
	// Prescaler 1024
	TCCR0B |= _BV(CS02) | _BV(CS00);
	
	// Overflow interrupt for Timer/Counter0 enabled
	TIMSK |= _BV(TOIE0);
	
	// External interrupt - any logical change
	MCUCR |= _BV(ISC00);
	
	// External interrupt for INT0 enabled
	GIMSK |= _BV(INT0);
	
	#elif DEVICE == 1
	// Normal mode
	// Prescaler 1024
	TCCR0A |= _BV(CS02) | _BV(CS00);
	
	// Overflow interrupt for Timer/Counter0 enabled
	TIMSK0 |= _BV(TOIE0);
	
	// External interrupt - any logical change
	EICRA |= _BV(ISC00);
	
	// External interrupt for INT0 enabled
	EIMSK |= _BV(INT0);
	
	#endif
	
	// Get the clearBufferTime time in ms (after this time, the buffer will be cleared)
	if (clearBufferTime < 83)
	{
		clearBTime = 83;
	}
	else
	{
		clearBTime = clearBufferTime;
	}
	
	// Initialize the buffer
	for (uint8_t i = 0; i < BUFFER_SIZE; i++)
	{
		durationBuffer[i] = 0;
	}
}

// Disables repetition of the entered systemCode and command (while holding a button, your instructions related to the systemCode and command are executed just once)
void IR_disableRepetition(uint8_t systemCode, uint8_t command)
{
	if(!isRepDisabled(systemCode, command))
	{
		numOfButtons++;
		// Dynamic memory allocation
		buttonsPtr = realloc(buttonsPtr, sizeof(*buttonsPtr) * 2 * numOfButtons);
		if (buttonsPtr != NULL)
		{
			*(buttonsPtr + 2 * (numOfButtons-1)) = systemCode;
			*(buttonsPtr + 2 * (numOfButtons-1) + 1) = command;
		}
	}
}

// Enables repetition of the entered systemCode and command, which were disabled earlier
void IR_enableRepetition(uint8_t systemCode, uint8_t command)
{
	// Iterates through whole dynamically allocated memory
	for (uint8_t i = 0; i < numOfButtons; i++)
	{
		if (*(buttonsPtr + (2 * i)) == systemCode && *(buttonsPtr + (2 * i) + 1) == command)
		{
			// Copy all buttons
			uint8_t *tempButtonsPtr = calloc(2 * numOfButtons, sizeof(*buttonsPtr));
			if (tempButtonsPtr != NULL)
			{
				for (uint8_t j = 0; j < 2 * numOfButtons; j++)
				{
					*(tempButtonsPtr + j) = *(buttonsPtr + j);
				}
				
				// Free buttons memory
				free(buttonsPtr);
				
				// Copy back all buttons except the deleted one
				numOfButtons--;
				buttonsPtr = calloc(2 * numOfButtons, sizeof(*buttonsPtr));
				if (buttonsPtr != NULL)
				{
					uint8_t k = 0;
					for (uint8_t j = 0; j < numOfButtons + 1; j++)
					{
						if (*(tempButtonsPtr + (2 * j)) != systemCode || *(tempButtonsPtr + (2 * j) + 1) != command)
						{
							*(buttonsPtr + (2 * k)) = *(tempButtonsPtr + (2 * j));
							*(buttonsPtr + (2 * k) + 1) = *(tempButtonsPtr + (2 * j) + 1);
							k++;
						}
					}
				}
				
				// Free temporary buttons memory
				free(tempButtonsPtr);
			}
			
			break;
		}
	}
}

// Returns true if receiving IR (initial pulses or repeat pulses)
bool IR_available()
{
	uint8_t systemCode = 0, address = 0, command = 0;
	static uint8_t lastSystemCode = 0, lastCommand = 0;
	static bool lastAvailable = false;
	
	// Clearing the buffer
	if (clearBuffer)
	{
		for (uint8_t i = 0; i < BUFFER_SIZE; i++)
		{
			durationBuffer[i] = 0;
		}
		clearBuffer = false;
	}

	// Checking the buffer
	if (bufferReady)
	{
		available = true;
		
		uint8_t index = 0, offset = 0;
		for (index = 0; index < 4 * 2; index += 2)
		{
			writeBit(index, offset, &systemCode);
		}
		offset += index;
		for (index = 0; index < 6 * 2; index += 2)
		{
			writeBit(index, offset, &address);
		}
		offset += index;
		for (index = 0; index < 8 * 2; index += 2)
		{
			writeBit(index, offset, &command);
		}
		
		IR.systemCode = systemCode;	// Assigning address to structure's systemCode member
		IR.address = address;		// Assigning address to structure's address member
		IR.command = command;		// Assigning command to structure's command member
		
		// Checks whether repetition is disabled
		if (IR.systemCode == lastSystemCode && IR.command == lastCommand && lastAvailable == true && isRepDisabled(IR.systemCode, IR.command))
		{
			bufferReady = false;
			return false;
		}
		// Repetition is allowed
		else
		{
			lastSystemCode = IR.systemCode;
			lastCommand = IR.command;
			bufferReady = false;
		}
	}
	// Checks whether repetition is disabled
	else if (available && isRepDisabled(IR.systemCode, IR.command))
	{
		return false;
	}
	
	lastAvailable = available;
	return available;
}

// Checks whether repetition is disabled
static bool isRepDisabled(uint8_t systemCode, uint8_t command)
{
	// Iterates through whole dynamically allocated memory
	for (uint8_t i = 0; i < numOfButtons; i++)
	{
		if (*(buttonsPtr + (2 * i)) == systemCode && *(buttonsPtr + (2 * i) + 1) == command)
		{
			return true;
		}
	}
	return false;
}

static void writeBit(uint8_t index, uint8_t offset, uint8_t *data)
{
	if (durationBuffer[++index + offset])
	{
		*data &= ~(1 << ((index - 1) / 2));
	} 
	else
	{
		*data |= (1 << ((index - 1) / 2));
	}
}

static void setSleepState()
{
	clearBuffer = true;
	state = SLEEP_STATE;
	level = true;
	bufferIndex = 0;
	nonSavingCounter = 0;
	available = false;
}

// External interrupt
ISR(INT0_vect)
{
	if (!bufferReady)
	{
		switch (state)
		{
			case SLEEP_STATE:
			state = LEADING_PULSE_STATE;
			break;
			
			// Leading 3.38 ms pulse
			case LEADING_PULSE_STATE:
			if (TICK * TCNT0 >= LEADING_PULSE - timeRange && TICK * TCNT0 <= LEADING_PULSE + timeRange)
			{
				state = LEADING_SPACE_STATE;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// Initial 1.69 ms space
			case LEADING_SPACE_STATE:
			if (TICK * TCNT0 >= INITIAL_SPACE - timeRange && TICK * TCNT0 <= INITIAL_SPACE + timeRange)
			{
				state = NON_SAVING_STATE;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// Bits which are not saved: 20 bits, 2 bits, 8 bits
			case NON_SAVING_STATE:
			// Short pulse or space
			if (TICK * TCNT0 >= LOGIC_SHORT - timeRange && TICK * TCNT0 <= LOGIC_SHORT + timeRange)
			{
				switch (nonSavingCounter)
				{
					case 39:
					state = SYSTEM_CODE_STATE;
					break;
					
					case 43:
					state = ADDRESS_STATE;
					break;
					
					case 59:
					state = FINAL_PULSE_STATE;
					break;
				}
				level = !level;
				nonSavingCounter++;
			}
			// Long space
			else if (TICK * TCNT0 >= LOGIC_LONG - timeRange && TICK * TCNT0 <= LOGIC_LONG + timeRange && !level)
			{
				switch (nonSavingCounter)
				{
					case 39:
					state = SYSTEM_CODE_STATE;
					break;
					
					case 43:
					state = ADDRESS_STATE;
					break;
					
					case 59:
					state = FINAL_PULSE_STATE;
					break;
				}
				level = !level;
				nonSavingCounter++;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// 4-bit system code
			case SYSTEM_CODE_STATE:
			// Short pulse or space
			if (TICK * TCNT0 >= LOGIC_SHORT - timeRange && TICK * TCNT0 <= LOGIC_SHORT + timeRange)
			{
				if (bufferIndex == 7)
				{
					state = NON_SAVING_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = true;
			}
			// Long space
			else if (TICK * TCNT0 >= LOGIC_LONG - timeRange && TICK * TCNT0 <= LOGIC_LONG + timeRange && !level)
			{
				if (bufferIndex == 7)
				{
					state = NON_SAVING_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = false;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// 6-bit address
			case ADDRESS_STATE:
			// Short pulse or space
			if (TICK * TCNT0 >= LOGIC_SHORT - timeRange && TICK * TCNT0 <= LOGIC_SHORT + timeRange)
			{
				if (bufferIndex == 19)
				{
					state = COMMAND_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = true;
			}
			// Long space
			else if (TICK * TCNT0 >= LOGIC_LONG - timeRange && TICK * TCNT0 <= LOGIC_LONG + timeRange && !level)
			{
				if (bufferIndex == 19)
				{
					state = COMMAND_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = false;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// 8-bit command
			case COMMAND_STATE:
			// Short pulse or space
			if (TICK * TCNT0 >= LOGIC_SHORT - timeRange && TICK * TCNT0 <= LOGIC_SHORT + timeRange)
			{
				if (bufferIndex == 35)
				{
					state = NON_SAVING_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = true;
			}
			// Long space
			else if (TICK * TCNT0 >= LOGIC_LONG - timeRange && TICK * TCNT0 <= LOGIC_LONG + timeRange && !level)
			{
				if (bufferIndex == 35)
				{
					state = NON_SAVING_STATE;
				}
				level = !level;
				durationBuffer[bufferIndex++] = false;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// Final 420 µs pulse
			case FINAL_PULSE_STATE:
			if (TICK * TCNT0 >= FINAL_PULSE - timeRange && TICK * TCNT0 <= FINAL_PULSE + timeRange)
			{
				clearBuffer = false;
				state = WAIT_STATE;
				ovfCounter = 1;
				level = true;
				bufferIndex = 0;
				nonSavingCounter = 0;
				bufferReady = true;
			}
			else
			{
				setSleepState();
			}
			break;
			
			// Waiting after final pulse
			case WAIT_STATE:
			state = LEADING_PULSE_STATE;
			break;
		}
	}
	
	TCNT0 = 0;
}

// Overflow interrupt
ISR(TIMER0_OVF_vect)
{
	// SLEEP_STATE begins after NUM_OF_OVFS overflow interrupts
	if (ovfCounter >= NUM_OF_OVFS)
	{
		state = SLEEP_STATE;
		level = true;
		bufferIndex = 0;
		nonSavingCounter = 0;
		available = false;
	}
	
	// Clearing buffer is allowed after NUM_OF_CLEAR_BUFFER_OVFS overflow interrupts
	if (ovfCounter >= NUM_OF_CLEAR_BUFFER_OVFS(clearBTime))
	{
		ovfCounter = 1;
		clearBuffer = true;
	}
	else
	{
		ovfCounter++;
	}
}