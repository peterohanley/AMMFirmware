#ifndef _ESCHAROTOMY_ARM_H_
#define _ESCHAROTOMY_ARM_H_

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdbool.h>

#include "Config/AppConfig.h"
#include "../../Timer.h"

MODULE_TASK(pulse);
MODULE_INIT(pulse);
PROX_HANDLER(pulse);
ACT_HANDLER(pulse);
INPUT_REQUESTEE(pulse);
void pulse_set_delay(int delay);
void pulse_stop(void);


#endif