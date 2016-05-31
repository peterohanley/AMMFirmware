#include "FlowSensor.h"

#define DOWN 0
#define UP 1
#define THRESH 512
#define TIME_THRESH 250
#define SENSOR_PIN 0
uint8_t last_fs_st;
ms_time_t last_blip;
//SomePString_t* error_to_send; // TODO
flow_messages last_flowmsg_rcvd = NO_FLOW_MESSAGE; //el_PROPOFOL;
uint8_t fs_module_st;
extern bool blip_msg_waiting;
bool iv_connected ;

//TODO don't store them twice if possible
FLOW_ACT_MESSAGE_TABLE(AS_RCV_STR);

void flow_enum_set_waiting(flow_messages k) {
#define AS_FLOW2ENUM_CASE(str) if (k == el_##str) {str##_msg_waiting = 1;return;}
	FLOW_ACT_MESSAGE_TABLE(AS_FLOW2ENUM_CASE);
	return;
}


void MODULE_TASK(flowsensor_task) //(uint16_t* adc_values) TODO
{
	ms_time_t now = millis();
	
	if ((last_fs_st == UP && adc_values[SENSOR_PIN] < THRESH) ||
		(last_fs_st == DOWN && adc_values[SENSOR_PIN] > THRESH)) {
		last_fs_st = !last_fs_st;
		
		blip_msg_waiting = 1;
		if (now <= (last_blip + TIME_THRESH)) {
			//fast! probably an iv!
			if (iv_connected && (last_flowmsg_rcvd != NO_FLOW_MESSAGE)) {
				flow_enum_set_waiting(last_flowmsg_rcvd);
				last_flowmsg_rcvd = NO_FLOW_MESSAGE;
			}
		}
		last_blip = now;
	}
	
	//now message sending logic
	/*
	if (blip_now && iv_connected && (last_flowmsg_rcvd != NO_FLOW_MESSAGE)) {
		flow_enum_set_waiting(last_flowmsg_rcvd);
		//last_flowmsg_rcvd = NO_FLOW_MESSAGE;
	}
	*/
	
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

void PROX_HANDLER(flow_sensor)
{
	uint8_t* data = Data; //FIXME
#define AS_ACT_CMP_CASE(s) if (!ram_prog_cmp(data,(char*)&pstr_rcv_##s,MIN(data[0],pstr_rcv_##s .len))) {last_flowmsg_rcvd = el_##s;return;} 
	
	FLOW_ACT_MESSAGE_TABLE(AS_ACT_CMP_CASE);
	//determine which string it is
	
}

//TODO this is defined somewhere else too, only define 
DEFINE_PSTRING(iv_connected_str, "ARM_R_IV_CATH");
void ACT_HANDLER(flow_sensor)
{
	uint8_t* data = Data; // FIXME
	//if ARM_R_IV_CATH is received, begin acknowledging IVs
	if (!ram_prog_cmp(data,(char*)&iv_connected_str,iv_connected_str.len)) {
		iv_connected = 1;
	}
}
