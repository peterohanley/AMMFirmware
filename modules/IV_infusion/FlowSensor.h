#ifndef _FLOWSENSOR_H_
#define _FLOWSENSOR_H_

#include "Config/AppConfig.h"
#include <avr/pgmspace.h>
#include <LUFA/Common/Common.h>
#include "Timer.h"

#define TIME_THRESH_DRUG 300
#define TIME_THRESH_DRIP 1000

#define FLOW_ACT_MESSAGE_TABLE(_) \
	_(ETOMIDATE, "GIVE_ETOMIDATE" ,TIME_THRESH_DRUG)\
	_(MORPHINE, "GIVE_MORPHINE", TIME_THRESH_DRUG)\
	_(FENTANYL, "GIVE_FENTANYL", TIME_THRESH_DRUG)\
	_(KETAMINE, "GIVE_KETAMINE", TIME_THRESH_DRUG)\
	_(ANTIBIOTICS, "GIVE_ANTIBIOTICS", TIME_THRESH_DRUG)\
	_(PROPOFOL, "GIVE_PROPOFOL", TIME_THRESH_DRUG)\
	_(LIDOCANE, "GIVE_LIDOCANE", TIME_THRESH_DRUG)\
	_(ROC, "GIVE_ROC", TIME_THRESH_DRUG)\
	_(FLUIDS, "ARM_FLUIDS_2L", TIME_THRESH_DRIP)\
	_(SUCCS, "ARM_L_SUCC_140MG", TIME_THRESH_DRUG)

#define AS_ACT_STR(s,str,tt) DEFINE_PSTRING(pstr_##s,str);
#define AS_RCV_STR(s,str,tt) DEFINE_PSTRING(pstr_rcv_##s,#s);
#define AS_ENUM_ELT(s,str,tt) el_##s,
#define AS_W8ING_BOOL(s,str,tt) bool s##_msg_waiting;
#define AS_HACK_DELAY_VAR(s,str,tt) ms_time_t HACK_msg_rcvd_##s;
#define AS_HACK_SENDABLE_VAR(s,str,tt) bool HACK_msg_sendable_##s;
typedef enum {
	NO_FLOW_MESSAGE,
	FLOW_ACT_MESSAGE_TABLE(AS_ENUM_ELT)
} flow_messages;


FLOW_ACT_MESSAGE_TABLE(AS_W8ING_BOOL);

FLOW_ACT_MESSAGE_TABLE(AS_HACK_DELAY_VAR);
FLOW_ACT_MESSAGE_TABLE(AS_HACK_SENDABLE_VAR);

bool blip_msg_waiting;
//bool fluids_sent;
ms_time_t last_blip;
bool dur_waiting;

void flowsensor_task(uint16_t* adc_values);
//void flow_enum_to_pstr(flow_messages k);
void flow_sensor_handle_PROX(char* data);
void flow_sensor_handle_ACT(char* data);
#endif