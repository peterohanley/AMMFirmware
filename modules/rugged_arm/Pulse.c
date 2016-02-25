#include "Pulse.h"

volatile uint8_t pulse_enabled;

MODULE_INIT(pulse)
{
	//set timer to fast pwm mode
	//page 131 for COM info 
	//see datasheet page 133 for WGM info
	//page 134 for CS (clock select) info
	TCCR1A = (1<<WGM11) | (1<<WGM10) | (1<<COM1B1);
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);
	
	//enable interrupts on timer
	//TIMSK1 = (1<<OCIE1A);
	
	//set TOP value to determine frequency
	OCR1A = 0xff;
	//freq = 16000000 * 256 = 62500Hz
	//set duty cycle
	OCR1B = 0x00;
	
	//enable output on appropriate pin (B6)
	DDRB |= (1<<PB6);
	
}


//the heartbeat could be done with an interrupt as well.
//plan for heartbeat
//make lookup table of one-byte duty cycle values for each quarter millisecond
//in the beat duration. Also have a value for the normal voltage (as a duty cycle, of course)
//not in PROGMEM because we're going to generate it dynamically
#define PULSE_NUM_ELTS 68
const uint8_t pulse[PULSE_NUM_ELTS] = 
	{0x2a,0x29,0x28,0x27,0x26,0x25,0x24,0x23,0x22,0x21,0x20,0x1f,0x28,0x33,0x3e,0x49,0x54,0x5f,0x6a,0x76,0x81,0x8c,0x97,0xa2,0xad,0xb8,0xc3,0xcf,0xda,0xe5,0xf0,0xfb,0xf9,0xef,0xe5,0xdb,0xd1,0xc7,0xbe,0xb4,0xaa,0xa0,0x96,0x8c,0x83,0x79,0x6f,0x65,0x5b,0x52,0x48,0x3e,0x34,0x2a,0x20,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2b,0x2c,0x2d}
;
#define DEFAULT_PULSE_LEVEL 31
ms_time_t last_loop_start;

INPUT_REQUESTEE(pulse)
{
	//TODO determine if a message should be sent
	//for pulse the answer is no
	UNUSED(Data);
	return 0;
}
int pulse_delay_ms = 1000 - PULSE_NUM_ELTS;
void pulse_start(void)
{
	last_loop_start = millis();
	pulse_enabled = 1;
}
void pulse_stop(void)
{
	pulse_enabled = 0;
}
void pulse_set_delay(int delay)
{
	pulse_enabled = 1;
	if (delay > PULSE_NUM_ELTS) {
		pulse_delay_ms = delay;
	} else {
		pulse_delay_ms = PULSE_NUM_ELTS; // FIXME 50% seems a good choice
	}
}
MODULE_TASK(pulse)
{
	if (pulse_enabled) {
		ms_time_t now = millis();
		ms_time_t offset = (now - last_loop_start) % pulse_delay_ms;
		uint8_t val = DEFAULT_PULSE_LEVEL;
		if (offset < PULSE_NUM_ELTS) {
			val = pulse[offset];
		}
		OCR1B = val;
	} else {
		OCR1B = 0;
	}
}

PROX_HANDLER(pulse)
{
	//TODO
	UNUSED(Data);
}

ACT_HANDLER(pulse)
{
	//TODO
	if (Data[1]=='S' && Data[2]=='T' && Data[3]=='O' && Data[4]=='P') { // skip length, check first character
		pulse_stop();
	}
}