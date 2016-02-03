#ifndef _TIMER_H_
#define _TIMER_H_
	#include <avr/io.h>
	#include <avr/interrupt.h>
	//#include <stdlib.h>
	//#include <sys/time.h>

	typedef uint32_t ms_time_t;
	typedef uint64_t TIME_t;
	//FIXME
	ms_time_t millis(void);
	TIME_t host_millis(void);
	void setup_timer(void);
	void set_time_oset(TIME_t);
	
	void time_to_wire(TIME_t t, uint8_t w[]);
	TIME_t time_from_wire(const uint8_t w[]);
#endif