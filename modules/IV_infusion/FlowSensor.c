#include "FlowSensor.h"

#define DOWN 0
#define UP 1
#define THRESH 512

ms_time_t time_thresh = TIME_THRESH_DRUG;
#define SENSOR_PIN 0
uint8_t last_fs_st;
ms_time_t last_blip;
//SomePString_t* error_to_send; // TODO
flow_messages last_flowmsg_rcvd = NO_FLOW_MESSAGE; //el_PROPOFOL;
#define MESSAGE_DELAY_TIME 10000
uint8_t fs_module_st;
extern bool blip_msg_waiting;
bool iv_connected;
//TODO don't store them twice if possible
FLOW_ACT_MESSAGE_TABLE(AS_RCV_STR);

void flow_enum_set_waiting(flow_messages k) {
#define AS_FLOW2ENUM_CASE(str,s,tt) if (k == el_##str) {str##_msg_waiting = 1;return;}
	FLOW_ACT_MESSAGE_TABLE(AS_FLOW2ENUM_CASE);
	return;
}


void flowsensor_task(uint16_t* adc_values)
{
	ms_time_t now = millis();
	
	if ((last_fs_st == DOWN && adc_values[SENSOR_PIN] > THRESH) ||
		(last_fs_st == UP && adc_values[SENSOR_PIN] < THRESH))
		 {
		last_fs_st = !last_fs_st;
		
		//blip_msg_waiting = 1;
		if (now <= (last_blip + time_thresh)) {
			//fast! probably an iv!
			if (iv_connected && (last_flowmsg_rcvd != NO_FLOW_MESSAGE)) {
				flow_enum_set_waiting(last_flowmsg_rcvd);
				last_flowmsg_rcvd = NO_FLOW_MESSAGE;
				time_thresh = TIME_THRESH_DRUG;
			}
		}
		last_blip = now;
	}
	
	//now message sending logic based on simple delays
#if 0
#define AS_DELAY_HANDLER(s,tt) do {\
	if (now >= (HACK_msg_rcvd_##s + MESSAGE_DELAY_TIME) && HACK_msg_sendable_##s && iv_connected) {\
		s##_msg_waiting = 1;\
		HACK_msg_sendable_##s = 0;\
	}\
	} while (0);
	FLOW_ACT_MESSAGE_TABLE(AS_DELAY_HANDLER);
#endif	
}


int ram_prog_cmp(char* ram, char* prog, int n)
{
	for (int i = 0; i < n; i++) {
		if (ram[i] < pgm_read_byte(&(prog[i]))) {
			return -1;
		} else if (ram[i] > pgm_read_byte(&(prog[i]))) {
			return 1;
		}
	}
	return 0;
}

void flow_sensor_handle_PROX(char* data)
{
	//real
	//#define AS_ACT_CMP_CASE(s) if (!ram_prog_cmp(data,(char*)&pstr_rcv_##s,MIN(data[0],pstr_rcv_##s .len))) {last_flowmsg_rcvd = el_##s;return;} 
	//hacky
#define AS_ACT_CMP_CASE(s,str,tt) if (!ram_prog_cmp(data,(char*)&pstr_rcv_##s,MIN(data[0],pstr_rcv_##s .len))) {last_flowmsg_rcvd = el_##s;time_thresh=tt;return;} 
	
	FLOW_ACT_MESSAGE_TABLE(AS_ACT_CMP_CASE);
	//determine which string it is
}

//TODO this is defined somewhere else too, only define 
DEFINE_PSTRING(iv_connected_str, "ARM_R_IV_CATH");
void flow_sensor_handle_ACT(char* data)
{
	//if ARM_R_IV_CATH is received, begin acknowledging IVs
	if (!ram_prog_cmp(data,(char*)&iv_connected_str,iv_connected_str.len)) {
		iv_connected = 1;
	}
}
