#ifndef _ESCHAROTOMY_ARM_H_
#define _ESCHAROTOMY_ARM_H_

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "Config/AppConfig.h"
#include "../../Timer.h"

#define ESCHAR_CUT_THRESH 200

#define MAKE_ESCHAR_MSG(x) DEFINE_PSTRING( eschar_msg_##x , "ARM_L_ESCHAR_" #x )

//MAKE_ESCHAR_MSG(1);
/*
MAKE_ESCHAR_MSG(2);
MAKE_ESCHAR_MSG(3);
MAKE_ESCHAR_MSG(4);
MAKE_ESCHAR_MSG(5);
*/

/* these are adc pin numbrs, in adc_values */
#define ESCHAR_PIN_1 6
#define ESCHAR_PIN_2 5
#define ESCHAR_PIN_3 4
#define ESCHAR_PIN_4 1
#define ESCHAR_PIN_5 0

/* when 4 is cut resume the pulse */
/* these pins form an L shape with the long arm under the skin 
eskinskinskinskinskin
l  3 2 1
b  4
o  5
wbonebonebonebonebone
*/
uint8_t eschar_msg_state_1;
uint8_t eschar_msg_state_2;
uint8_t eschar_msg_state_3;
uint8_t eschar_msg_state_4;
uint8_t eschar_msg_state_5;
#define ESCHAR_MSG_WAITING 1
#define ESCHAR_MSG_SENT 2
// 0 means not yet detected

void eschar_init(void);
void eschar_task(uint16_t* adc_values);
void pulse_task(void);
void pulse_set_delay(int delay);
void pulse_stop(void);
void pulse_init(void);

void heat_enable(void);

#endif