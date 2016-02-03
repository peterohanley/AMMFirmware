#ifndef _FLOWSENSOR_H_
#define _FLOWSENSOR_H_

#include "Config/AppConfig.h"
#include <avr/pgmspace.h>
#include <LUFA/Common/Common.h>
#include "Timer.h"


#define FLOW_ACT_MESSAGE_TABLE(_) \
	_(ETOMIDATE)\
	_(MORPHINE)\
	_(FENTANYL)\
	_(KETAMINE)\
	_(ANTIBIOTICS)\
	_(PROPOFOL)\
	_(LIDOCANE)\
	_(ROC)\
	_(FLUIDS)

#define AS_ACT_STR(s) DEFINE_PSTRING(pstr_##s,"GIVE_" #s);
#define AS_RCV_STR(s) DEFINE_PSTRING(pstr_rcv_##s,#s);
#define AS_ENUM_ELT(s) el_##s,
#define AS_W8ING_BOOL(s) bool s##_msg_waiting;

typedef enum {
	NO_FLOW_MESSAGE,
	FLOW_ACT_MESSAGE_TABLE(AS_ENUM_ELT)
} flow_messages;


FLOW_ACT_MESSAGE_TABLE(AS_W8ING_BOOL);

bool blip_msg_waiting;
bool fluids_sent;
ms_time_t last_blip;
bool dur_waiting;

void flowsensor_task(uint16_t* adc_values);
//void flow_enum_to_pstr(flow_messages k);
void flow_sensor_handle_PROX(char* data);
void flow_sensor_handle_ACT(char* data);
#endif