#include "Esophagus.h"
#include "Config/AppConfig.h"

//put esophagus code here TODO

/* GAS THRESHOLD DETECTOR VARIABLES */
//TODO these will be 2 seperate modules, so make them function correctly

//stomach
DEFINE_PSTRING(esophageal_msg_str, "ESOPHAGEAL_VENTILATION_ET_TUBE");
ms_time_t stomach_last_sent_event;
#define stomach_adc_pin 7
uint8_t esophageal_msg_waiting;
uint8_t eso_st;
ms_time_t eso_flow_stop_time;
bool stomach_evt_occuring;





#define right_bronchus_adc_pin 5
bool right_bronchus_evt_occuring;

#define left_bronchus_adc_pin 7
bool left_bronchus_evt_occuring;
ms_time_t lung_flow_stop_time;
unsigned char lung_st;
unsigned char vent_msg_waiting;
DEFINE_PSTRING(vent_msg_str,"VENTILATION_ET_TUBE");
unsigned char bvm_off_msg_waiting;
DEFINE_PSTRING(bvm_off_msg_str,"BVM_OFF");
unsigned char mainstem_msg_waiting;
DEFINE_PSTRING(mainstem_msg_str, "MAINSTEM_VENTILATION_ET_TUBE");
unsigned char hypervent_msg_waiting;
DEFINE_PSTRING(hypervent_msg_str, "MASK_HYPERVENTILATE_PT");
bool mask_main_exclusion;

#define GAS_EVENT_WAIT_TIME 1000
#define WAIT_FOR_BOTH_PRESSURES_MS 3000
#define BVM_OFF_WAIT_TIME_MS 5000

#define GAS_PRESSURE_LEARN_TIME_MS 1500
bool gas_pressure_learned;
ms_time_t gas_pressure_learning_started;
uint16_t gas_pressure_threshold = 1023;

#define ACT_MSG_ST_RCVD 1
uint8_t bvm_sitch, vent_sitch;


void lung_module_init(void) {
	gas_pressure_learning_started = millis();
}

void lung_module_task(void)
{

	ms_time_t now = millis();
	//learn for a while
	if (!gas_pressure_learned && (now 
		>= (gas_pressure_learning_started + GAS_PRESSURE_LEARN_TIME_MS))) {
		gas_pressure_learned = 1;
		gas_pressure_threshold	= adc_values[stomach_adc_pin] + 5;
	}
	bool eso_press = gas_pressure_learned ?
		  adc_values[stomach_adc_pin] >= gas_pressure_threshold
		: 0;
	if (!bvm_sitch) {
		eso_st = 0;
	} else {
		if (eso_press) {
			if (eso_st == 0 || eso_st == 1) {
				esophageal_msg_waiting = 1;
				eso_st = 3;
			}
		} else {
			if (eso_st == 3) {
				eso_flow_stop_time = millis();
				eso_st = 2;
			} else if (eso_st == 0) {
				eso_st = 1;
			} else if (eso_st == 2) {
				if (now - eso_flow_stop_time > BVM_OFF_WAIT_TIME_MS) {
					bvm_off_msg_waiting = 1;
					bvm_sitch = 0;
				}
			}
		}
	}
}
