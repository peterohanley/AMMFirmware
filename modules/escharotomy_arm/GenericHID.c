/*
             LUFA Library
     Copyright (C) Dean Camera, 2015.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2015  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the GenericHID demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "GenericHID.h"

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevHIDReportBuffer[BIO_EVENT_REPORT_SIZE]; 

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Generic_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = INTERFACE_ID_GenericHID,
				.ReportINEndpoint             =
					{
						.Address              = GENERIC_IN_EPADDR,
						.Size                 = GENERIC_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevHIDReportBuffer),
			},
	};


/* the bootloader stuff */
uint32_t Boot_Key ATTR_NO_INIT;
void Bootloader_Jump_Check(void)
{
    // If the reset source was the bootloader and the key is correct, clear it and jump to the bootloader
    if ((MCUSR & (1 << WDRF)) && (Boot_Key == MAGIC_BOOT_KEY))
    {
		//overwrite boot key so that if the code that's loaded used the same location and value it will not re-bootload.
		//MCUSR &= 0 ; //~(1 << WDRF);
        Boot_Key = 0;
        ((void (*)(void))BOOTLOADER_START_ADDRESS)();
    }
}

void Jump_To_Bootloader(void)
{
    // If USB is used, detach from the bus and reset it
    //USB_Disable();
	USB_Detach();
    // Disable all interrupts
    cli();
    // Wait two seconds for the USB detachment to register on the host
    Delay_MS(1500);
    // Set the bootloader key to the magic value and force a reset
    Boot_Key = MAGIC_BOOT_KEY;
    wdt_enable(WDTO_500MS);
    for (;;);
}

/* end bootloader stuff*/


#define DEFAULT_SENSOR_THRESH 200.0
float sensor_evt_thresh[4] = {
	  DEFAULT_SENSOR_THRESH
	, DEFAULT_SENSOR_THRESH
	, DEFAULT_SENSOR_THRESH
	, DEFAULT_SENSOR_THRESH
};
#undef DEFAULT_SENSOR_THRESH
float sensor_varnces[4];

#define EVENT_T_INIT {0,0,0}
event_buf_t event_buffer = {10,0,0,0,
	{EVENT_T_INIT
	,EVENT_T_INIT,EVENT_T_INIT,EVENT_T_INIT
	,EVENT_T_INIT,EVENT_T_INIT,EVENT_T_INIT
	,EVENT_T_INIT,EVENT_T_INIT,EVENT_T_INIT
	}
};
#undef EVENT_T_INIT


/* hold the state for the string descriptor hack */
#define STRINGSTM_STATE_BASE 0
#define STRINGSTM_STRING_REQUESTED 1
uint8_t stm_state = STRINGSTM_STATE_BASE;
uint8_t stm_reqd_stringid = 0;
uint8_t stm_reqd_offset = 0;

uint16_t adc_values[12];

uint16_t adc_read(int pin) {
	uint8_t lo, hi;
	uint8_t pinlo = pin & 0x1f;
	
	bool pinhi = pin & 0x20;
	if (pinhi) { //TODO optimize away the if
		ADCSRB |= (1<<MUX5);
	} else {
		ADCSRB &= ~(1<<MUX5);
	}
	
	//ADMUX &= 0xf8; //clear low pins
	//ADMUX |= pinlo;
	ADMUX = (ADMUX & 0xe0) | pinlo; //clear low pins
	
	ADCSRA |= (1<<ADSC); //get an adc value
	while (ADCSRA & (1<<ADSC)); //Wait for it to do the adc
	lo = ADCL;
	hi = ADCH;
	return (hi << 8) | lo;
}

void adc_task(void) {
	int i;
	for (i = 0; i < 9; i++) {
		adc_values[i] = adc_read(i);
	}
}

//to make it utf-16, add L"" before BIO_REPORT_TABLE and s/char/int/
DEFINE_PSTRING(bio_report_string,BIO_REPORT_TABLE(BIO_AS_REPORT_STRING));

#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)

/* RFID CODE VARIABLES */
bool parsed_rfid_ready;

/* FLOW RATE VARIABLES */
//TODO

/* GAS THRESHOLD DETECTOR VARIABLES */
//TODO these will be 2 seperate modules, so make them function correctly

#define EVT_DOWN 1
#define EVT_UP 2
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

uint16_t gas_pressure_threshold = 505;
#define GAS_EVENT_WAIT_TIME 1000
#define WAIT_FOR_BOTH_PRESSURES_MS 3000
#define BVM_OFF_WAIT_TIME_MS 3000

#define ACT_MSG_ST_RCVD 1
uint8_t bvm_sitch, vent_sitch;

#define ESOPHAGEAL_ST_1 1
uint8_t esophageal_st;

//workaround
uint8_t o2_msg_waiting;
DEFINE_PSTRING(o2_msg_str,"O2");


#define GAS_IS_LUNGS
void lung_module_task(void)
{
	//FIXME see if these if branches can be neater
#ifdef GAS_IS_LUNGS
	/*
	bool leftpress = adc_values[left_bronchus_adc_pin] >= gas_pressure_threshold;
	bool rightpress = adc_values[right_bronchus_adc_pin] >= gas_pressure_threshold;
	if (!right_bronchus_evt_occuring) {
		if (rightpress) {
			right_bronchus_evt_occuring = 1;
			if (leftpress) {
				//send success message
				bronchus_success_evt_waiting = 1;
				left_bronchus_evt_occuring = 1;
			} else {
				//send failure message
				left_bronchus_evt_occuring = 0;
				bronchus_mainstem_evt_waiting = 1;
			}
		}
	} else if (adc_values[right_bronchus_adc_pin] < gas_pressure_threshold) {
		right_bronchus_evt_occuring = 0;
	}
	*/
	bool left_press = adc_values[left_bronchus_adc_pin] >= gas_pressure_threshold;
	bool right_press = adc_values[right_bronchus_adc_pin] >= gas_pressure_threshold;
	bool anymsg = bvm_sitch || vent_sitch;
	if (!anymsg) {
		lung_st = 0;
	} else if (!right_press && !left_press) {
		if (lung_st == 0) {
			lung_st = 1;
			//no message
		} else if (lung_st == 3 || lung_st == 4 || lung_st == 5) {
			lung_flow_stop_time = millis();
			lung_st = 2;
		} else if (lung_st == 2) {
			ms_time_t now = millis();
			if (now - lung_flow_stop_time > BVM_OFF_WAIT_TIME_MS) {
				bvm_off_msg_waiting = 1;
				//re-enable mask/mainstem
				mask_main_exclusion = 0;
				bvm_sitch = 0;
				lung_st = 1;
			}
		}
	} else if (right_press && !left_press && bvm_sitch) {
		if ((lung_st == 0 || lung_st == 1 || lung_st == 3) && !mask_main_exclusion) {
			lung_st = 4;
			mainstem_msg_waiting = 1;
			mask_main_exclusion = 1;
		}
	} else if (right_press && left_press) {
		if (vent_sitch) {
			vent_msg_waiting = 1;
			lung_st = 5;
		} else if (bvm_sitch) {
			if ((lung_st == 0 || lung_st == 1 || lung_st == 4) && !mask_main_exclusion) {
				hypervent_msg_waiting = 1;
				mask_main_exclusion = 1;
				lung_st = 3;
			}
		}
	}
	
#else
	bool eso_press = adc_values[stomach_adc_pin] >= gas_pressure_threshold;
	/*
	if (stomach_evt_occuring) {
		if (adc_values[stomach_adc_pin] < gas_pressure_threshold) {
			stomach_evt_occuring = 0; // don't send again
			stomach_evt_waiting = 0; // don't send down message 
		}
	} else {
		if (adc_values[stomach_adc_pin] > gas_pressure_threshold) {
			stomach_evt_occuring = 1;
			stomach_evt_waiting = EVT_UP;
		}
	}
	*/
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
				ms_time_t now = millis();
				if (now - eso_flow_stop_time > 2000) {
					bvm_off_msg_waiting = 1;
					bvm_sitch = 0;
				}
			}
		}
	}
#endif
}

/* code for echoing PROX to ACT */
unsigned char prox2act[MIN(PROX_REPORT_SIZE,BIO_EVENT_REPORT_SIZE)];
//#define ECHO_PROX_TO_ACT
bool prox2act_msg_waiting;
/* ACT macro*/
#define SEND_ACT(pstr) do {int len;\
						const char* str;\
						len = pstr.len;\
						len = MIN(len, BIO_EVENT_REPORT_SIZE);\
						str = pstr.content;\
						Data[0] = len;\
						for (int i = 0;i < len; i++) {\
							Data[1+i] = pgm_read_byte(str + i);\
						}\
						*ReportID = BIO_EVENT_REPORT_ID;\
						*ReportSize = BIO_EVENT_REPORT_SIZE; } while (0)
#define SEND_PROX_UGLY_HACK(pstr_len,pstr_content) do {unsigned char len;\
						const char* str;\
						len = MIN(pstr_len, BIO_EVENT_REPORT_SIZE);\
						str = pstr_content;\
						Data[0] = len;\
						for (unsigned char i = 0;i < len; i++) {\
							Data[1+i] = pgm_read_byte(str+i);\
						}\
						*ReportID = PROX_REPORT_ID;\
						*ReportSize = PROX_REPORT_SIZE; } while (0)
/* ESCHAROTOMY ARM VARIABLES. Here because they must be initialized, as they are
 in PROGMEM, but they can't be declared twice */
MAKE_ESCHAR_MSG(1);
MAKE_ESCHAR_MSG(2);
MAKE_ESCHAR_MSG(3);
MAKE_ESCHAR_MSG(4);
MAKE_ESCHAR_MSG(5);

/* IV ARM VARIABLES */

DEFINE_PSTRING(iv_arm_msg, "ARM_R_IV_CATH");

/* DEVICE NAME */
DEFINE_PSTRING(device_name_string, "escharotomy_arm");

/* DEBUG FLOW SENSOR */
DEFINE_PSTRING(blip_str,"BLIP");

/* DEBUG ESCHAR ARM */
DEFINE_PSTRING(heat_str,"HEAT");
bool heat_msg_waiting;
/* FLOW SENSOR VARIABLES */
FLOW_ACT_MESSAGE_TABLE(AS_ACT_STR);


/* PIN BUMP DETECTOR VARIABLES */

typedef enum e_pinstmstate {
	PS_NO_CONTACT,
	PS_DURING_CONTACT
} pin_stm_state;
pin_stm_state pstm_st = PS_NO_CONTACT;
TIME_t pin7_start_time_host;
ms_time_t pin7_start_time;
//time_t pin7_end_time; /* probably not useful */
bool pin7_evt_avail;
TIME_t pin7_evt_start_host;
ms_time_t pin7_evt_dur;
void pin7_task(void)
{
	bool pin7on = ! (PINE & (1<<PE6));
	ms_time_t pin7_end_time;
	switch (pstm_st) {
		case PS_NO_CONTACT:
		if (pin7on) {
			/* TODO maybe ensure these have the same millisecond value */
			pin7_start_time_host = host_millis();
			pin7_start_time = millis();
			pstm_st = PS_DURING_CONTACT;
		} else {
			/* no action */
		}
		break;
		case PS_DURING_CONTACT:
		if (pin7on) {
			/* no action */
		} else {
			pin7_end_time = millis();
			pstm_st = PS_NO_CONTACT;
			pin7_evt_start_host = pin7_start_time_host;
			pin7_evt_dur = pin7_end_time - pin7_start_time;
			pin7_evt_avail = 1;
		}
	}
}
/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();
	setup_timer();
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();
	//setup_airwaysensor();
	//Serial_Init(9600, 0);
	eschar_init();
	//pulse_init(); // moved, so that pulse does not start immediately
	//rfid_init();

	for (;;)
	{
		HID_Device_USBTask(&Generic_HID_Interface);
		USB_USBTask();
		adc_task();
		//airwaysensor_task(adc_values, sensor_varnces, sensor_evt_thresh, &event_buffer);
		//pin7_task();
		//lung_module_task();
		eschar_task(adc_values);
		pulse_task();
		//rfid_task();
		//parsed_rfid_ready = try_parse_message();
		
		//flowsensor_task(adc_values);
	}
}



/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
#if 0
	/* disable port f pullups */
	PORTF = 0x00;
	/* make port F input */
	DDRF = 0x00;
	/* disable port f pullups */
	PORTF = 0x00;
	
	
	/* disable all pullups */
	MCUCR |= (1u << PUD);
	
	/* disable JTAG hopefully FOR REAL */
	MCUCR |= (1<<JTD);
	MCUCR |= (1<<JTD); // you have to do it twice
	
	/* disable digital input buffer */
	DIDR1 &= ~(1<<AIN0D);
	DIDR0 |= 0xff;
#endif
	/* enable and configure ADC */
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); /* sets clock division */ 
	ADCSRA |= (1<<ADEN); /*enable*/
	/* to select channel, write to ADMUX */
	//ADCSRB |= MUX5;
	//ADMUX = 2; /* ADC channel 10? */

	ADMUX = (1<<REFS0) /*| (1<<REFS1)*/; /* set ref, arduino uses 1<<6 */

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
	ADCSRA |= (1<<ADSC); /* do the longer first conversion */
	
	/* set pin 7 (on schematic: D7 = PE6) as input */
	/* all pins are input by default */
	/* enable pullup resistor */
	PORTE |= (1<<PE6);
	
	/* enable pins as output */
	DDRB |= (1<<PB1) | (1<<PB2) | (1<<PB3);
	//DDRC |= (1<<PC6) | (1<<PC7); 
	
	//from e arm, ove later
	DDRB |= (1<<PB6) | (1<<PB7);
	
	
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Generic_HID_Interface);

	USB_Device_EnableSOFEvents();

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Generic_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Generic_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	uint8_t* Data        = (uint8_t*)ReportData;
	//uint8_t  CurrLEDMask = LEDs_GetLEDs();
	//uint16_t cb_wValue;
	//const USB_Descriptor_String_t* str_addr = NULL;
	//const void** const cb_descriptoraddr = (void*) &str_addr;
	//const uint8_t* strchars;
	//uint16_t cb_strlen;
	//uint16_t strlen_rem;
	TIME_t ms;
	UNUSED(HIDInterfaceInfo);
	const event_t* nextevt;
	switch (ReportType) {
		case HID_REPORT_ITEM_Feature:
			/*if (stm_state == STRINGSTM_STRING_REQUESTED 
				&& *ReportID == STRING_DESC_HACK_REPORT_ID) { 
				cb_wValue = (DTYPE_String << 8) | stm_reqd_stringid;
				// use the descriptor callback to get the string ptr and length, then index with the offset
				cb_strlen = CALLBACK_USB_GetDescriptor(cb_wValue, 0, cb_descriptoraddr);
				//put bytes of string into buffer
				if (stm_reqd_offset < cb_strlen) {
					Data[0] = cb_strlen >> 8;
					Data[1] = cb_strlen & 0xff;
					strlen_rem = cb_strlen - stm_reqd_offset;
					Data[2] = strlen_rem >> 8;
					Data[3] = strlen_rem & 0xff;
					Data+=4;
					strchars = (uint8_t*) str_addr->UnicodeString;
					strchars += stm_reqd_offset;
					for (int i = 0; i <30; i++) {
						Data[i]=pgm_read_byte(strchars + i);
					}
					Data -= 4;
				}
				
				*ReportSize = STRING_DESC_REPORT_HACK_SIZE;
				stm_state = STRINGSTM_STATE_BASE;
				return true;
			} else */
			if (*ReportID == TIMESTAMP_OFFSET_FR_ID) {
				//return the current timestamp
				ms = host_millis();
				time_to_wire(ms, Data);
				*ReportSize = TIMESTAMP_FR_SIZE;
				return true;
			} else if (*ReportID == SET_SENS_THRESH_REPORT_ID) {
				for (int i = 0; i < 4; i++) {
					float_to_wire(sensor_evt_thresh[i], Data + 4*i);
				}
				*ReportSize = SET_SENS_THRESH_REPORT_SIZE;
				return true;
			} else if (*ReportID == REPORT_MAP_STRING_ID) {
				int len = bio_report_string.len;
				len = len > 0xfe ? 0xfe : len;
				Data[0] = len;
				for (int i = 0; i < len; i++) {
					Data[1+i] = pgm_read_byte(bio_report_string.content + i);
				}
				*ReportSize = REPORT_MAP_STRING_SIZE;
				return true;
			} else if (*ReportID == DEVICE_NAME_REPORT_ID) {
				int len = device_name_string.len;
				len = len > 0xfe ? 0xfe : len;
				Data[0] = len;
				for (int i = 0; i < len; i++) {
					Data[1+i] = pgm_read_byte(device_name_string.content + i);
				}
				*ReportSize = DEVICE_NAME_REPORT_SIZE;
				return true;
			} else if (*ReportID == NO_DATA_REPORT_ID) {
				Data[0] = lung_st;
				*ReportSize = NO_DATA_REPORT_SIZE;
				return true;
			} else { /* make some other kind of feature report */
				return false;
			}
		
			break;
		case HID_REPORT_ITEM_In:
		//FIXME use another switch here rather than elif cascade
		/*
		Here we must decide what to send back. If there is a waiting event 
		(pin7, ADC) send that.
		
		Otherwise send the next bio report.
		
		However, every report id must also have a clause for if it is specicifically requested. 
		*/
		//TODO use a table of functions that might set the report, call each in turn
		//this will allow better operation with multiple reports, and allow each module's variables to be encapsulated in their files
			
			if ((*ReportID == GET_SENSOR_EVENT_REPORT_ID) /*(*ReportID == 0)*/) {
				//FIXME duplicated code here. Not sure how to handle this duplication as this code must be on the control path when this specific report is requested.
				if (event_buffer.occupancy) { // nextevt != NULL) {
					
					nextevt = deq_event(&event_buffer);
					event_to_wire(nextevt, Data);
					*ReportSize = GET_SENSOR_EVENT_REPORT_SIZE;
					*ReportID = GET_SENSOR_EVENT_REPORT_ID;
					return true;
				} else {
					return false;
				}
			} else if ((*ReportID == 3) /*|| (*ReportID == 0)*/) { //show adc output
				int desired[] = {1, 4, 6, 7}; // 1467 is real, use bogus only for checking
				for (int adcix = 0; adcix < 4; adcix++) {
					Data[2*adcix + 1] = (adc_values[desired[adcix]] >> 8);
					Data[2*adcix] = adc_values[desired[adcix]] & 0xff;
				}
				for (int i = 0; i < 4; i ++) {
					float_to_wire(sensor_varnces[i],Data+8+4*i);
				}
				*ReportID = 3;
				*ReportSize = 24;
				return true;
			} else  if (*ReportID == 0) {
				
				if (rfid_usable_to_send) {
					char *msgbuf;
					msgbuf = *usable_message;
					if (msgbuf[0] == 0x44 && msgbuf[1] == 0x16) {
						unsigned char* tagstart = (unsigned char*) msgbuf+10;
						unsigned char pstr_len;
						const char* pstr_msg = NULL;
						
						if (classify_tag(tagstart, &pstr_len, &pstr_msg)) {
							if (pstr_msg == NULL) {
								//Recognized, but sent recently. take no action.
								return false;
							} else {
								rfid_enable_buzzer();
								SEND_PROX_UGLY_HACK(pstr_len,pstr_msg);
								rfid_usable_to_send = 0;
								return true;
							}
						} // classify_tag returns 0, tag not recognized
					}
					return false;
					/* // passthrough code
					if (msgbuf[0] == 0x44 && msgbuf[1] == 0x05) {
						rfid_usable_to_send = 0;
						return false;
					}
					for (int i = 0; i < 64; i++) {
						Data[i] = msgbuf[i];
					}
					
					*ReportID = RFID_TAG_PASSTHROUGH_REPORT_ID;
					*ReportSize = 64; //FIXME if this ever changes gotta fix it
					rfid_usable_to_send = 0;
					return true;
					//*/
				}
#ifdef ECHO_PROX_TO_ACT 
				else if (prox2act_msg_waiting) {
					int p2a_len = prox2act[0];
					for (int i = 0; i <= p2a_len; i++) {
						Data[i] = prox2act[i];
					}
					*ReportID = BIO_EVENT_REPORT_ID;
					*ReportSize = BIO_EVENT_REPORT_SIZE;
					prox2act_msg_waiting = 0;
					return true;
				}
#endif
				else if (esophageal_msg_waiting) {
					SEND_ACT(esophageal_msg_str);
					esophageal_msg_waiting = 0;
					return true;
				} else if (vent_msg_waiting) {
					SEND_ACT(vent_msg_str);
					vent_msg_waiting = 0;
					return true;
				} else if (bvm_off_msg_waiting) {
					SEND_ACT(bvm_off_msg_str);
					bvm_off_msg_waiting = 0;
					return true;
				} else if (mainstem_msg_waiting) {
					SEND_ACT(mainstem_msg_str);
					mainstem_msg_waiting = 0;
					return true;
				} else if (hypervent_msg_waiting) {
					SEND_ACT(hypervent_msg_str);
					hypervent_msg_waiting = 0;
					return true;
				} else if (o2_msg_waiting) {
					SEND_ACT(o2_msg_str);
					o2_msg_waiting = 0;
					return true;
				} else if (blip_msg_waiting) {
					SEND_ACT(blip_str);
					blip_msg_waiting = 0;
					return true;
				} else if (heat_msg_waiting) {
					SEND_ACT(heat_str);
					heat_msg_waiting = 0;
					return true;
				}
				//TODO
#define AS_ACT_SENDER(s) \
	if (s##_msg_waiting) {\
		SEND_ACT(pstr_##s);\
		s##_msg_waiting = 0;\
		return true;\
	}
				FLOW_ACT_MESSAGE_TABLE(AS_ACT_SENDER);
				//FIXME use an array and a loop
#define ESCHAR_RESPONSE_MACRO(x) if (eschar_msg_state_##x == ESCHAR_MSG_WAITING) {\
					SEND_ACT(eschar_msg_##x);\
					eschar_msg_state_##x = ESCHAR_MSG_SENT;\
					return true;\
				}
				ESCHAR_RESPONSE_MACRO(1)
				ESCHAR_RESPONSE_MACRO(2)
				ESCHAR_RESPONSE_MACRO(3)
				ESCHAR_RESPONSE_MACRO(4)
				ESCHAR_RESPONSE_MACRO(5)
				
				//send the sensor spike event
				if (event_buffer.occupancy) {
					//if we are an iv arm, rather than sending a spike event send a message on ACT: "ARM_R_IV_CATH"
					if (RV_IS_IV_ARM) {
						deq_event(&event_buffer); //intentionally discard
						SEND_ACT(iv_arm_msg);
						return true;
					} else { // some other thing that uses the rv code FIXME list them
						nextevt = deq_event(&event_buffer);
						event_to_wire(nextevt, Data);
						*ReportSize = GET_SENSOR_EVENT_REPORT_SIZE;
						*ReportID = GET_SENSOR_EVENT_REPORT_ID;
						LEDs_SetAllLEDs(LEDMASK_USB_ERROR);
						return true;
					}
				//pin touch event
				} else if (pin7_evt_avail) {
					ms_time_t evt_dur = pin7_evt_dur;
					ms = pin7_evt_start_host;
					time_to_wire(ms, Data);
					uint32_to_wire(evt_dur,Data+8);
					pin7_evt_avail = 0;
					*ReportID = WIRE_CONTACT_REPORT_ID;
					*ReportSize = WIRE_CONTACT_REPORT_SIZE;
					return true;//pin touch event
				//send the bio report
					//} else if (real bio report event) {
				
				}
				#if 1
				else {
					if (SEND_ADC_DATA) {
						//send sensor
						int desired[] = RV_STM_ADC_NUMS;
						for (int adcix = 0; adcix < RV_STM_COUNT; adcix++) {
							Data[2*adcix + 1] = (adc_values[desired[adcix]] >> 8);
							Data[2*adcix] = adc_values[desired[adcix]] & 0xff;
						}
						for (int i = 0; i < RV_STM_COUNT; i ++) {
							float_to_wire(sensor_varnces[i],Data+(2*RV_STM_COUNT)+4*i);
						}
						*ReportID = 3;
						*ReportSize = (RV_STM_COUNT*(4+2));
						return true;
					}
				}
				#endif
				/*
				else {
					Data[0] = eso_st;
					*ReportID = NO_DATA_REPORT_ID;
					*ReportSize = NO_DATA_REPORT_SIZE;
					return true;
				}
				*/
			}
	}
	return false;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	uint8_t* Data       = (uint8_t*)ReportData;
	//uint8_t  NewLEDMask = LEDS_NO_LEDS;
	UNUSED(HIDInterfaceInfo);
	UNUSED(ReportSize);
	switch (ReportType) {
		case HID_REPORT_ITEM_Feature:
		//store string id and offset in alotted bytes
		//use the getDescriptor callback to get the string pointer and length
			if (ReportID == STRING_DESC_HACK_REPORT_ID) {
				stm_state = STRINGSTM_STRING_REQUESTED;
				stm_reqd_stringid = Data[0];
				stm_reqd_offset = Data[1];
			} else if (ReportID == TIMESTAMP_OFFSET_FR_ID) {
				TIME_t oset;
				oset = time_from_wire(Data);
				set_time_oset(oset);
			} else if (ReportID == SET_SENS_THRESH_REPORT_ID) {
				for (int i=0; i < 4; i++) {
					sensor_evt_thresh[i] = float_from_wire(Data + 4*i);
				}
			} else if (ReportID == START_BOOTLOADER_REPORT_ID) {
				//check that proper code was supplied
				//FIXME lol always succeed
				//start bootloader
				LEDs_SetAllLEDs(LEDS_LED1|LEDS_LED2|LEDS_LED3);
				Jump_To_Bootloader();
			} else if (ReportID == NO_DATA_REPORT_ID) {
				heat_enable();
				heat_msg_waiting = 1;
			}
			break;
		case HID_REPORT_ITEM_Out:
		//TODO have these handlers update the value sent on the corresponding INPUT reports.
		
		switch (ReportID) {
			
			case RFID_TAG_SCAN_COMMAND_REPORT_ID:
			//rfid_parser_clearbuffers();
			if (Data[0] == 0) {
				rfid_sendcommand_readtags();
			}
			//TODO do whatever else needs to be set up
			break;
			
			case PROX_REPORT_ID:;
			//TODO check message, take appropriate action
			/* 
			Airway module must
				consume
					BVM
					VENT
			Stomach module must also consume BVM.
			*/
			/*
			int len = Data[0];
			for (int i = 0; i <= len; i++) {
				Serial_SendByte(Data[i]);
			}*/
#ifdef ECHO_PROX_TO_ACT
			//int len = Data[0];
			for (int i = 0; i <= len; i++) {
				prox2act[i] = Data[i];
			}
			prox2act_msg_waiting = 1;
#endif
			if (Data[1] == 'V' && Data[2] == 'E' && Data[3] == 'N' && Data[4] == 'T') { // VENT
				//TODO what is the appropriate action here
				vent_sitch = ACT_MSG_ST_RCVD;
			} else if (Data[1] == 'B' && Data[2] == 'V' && Data[3] == 'M') { // BVM
				//TODO what is the appropriate action here
				//do not send any airway messages if the BVM message has not been received
				bvm_sitch = ACT_MSG_ST_RCVD;
				//debug_msg_waiting = 1;
			} else if (Data[1] == 'O' && Data[2] == '2') {
				//airway emits an O2 message on ACT, echoing it
				//makes rfid code simpler
				o2_msg_waiting = 1;
			}
			flow_sensor_handle_PROX((char*)Data);
			break;
			
			case BIO_EVENT_REPORT_ID:; /* ACT */
			//TODO check message, take appropriate action
			/*especially be sure to handle STOP message
				STOP message must be received by:
				Escharotomy arm
				IV arm
				Rugged arm
				or rather, all the arms that have a pulse.
			*/
			if (Data[1]=='S' && Data[2]=='T' && Data[3]=='O' && Data[4]=='P') { // skip length, check first character
				pulse_stop();
			}
			flow_sensor_handle_ACT((char*) Data);
			break;
			
			case HEART_RATE_REPORT_ID:;
			//adjust rate of the pulse generator
			//do this by changing the dead time on each pulse
			/* needed by:
				Rugged Arm
				Escharotomy Arm
				IV Arm
			*/
			int pulse_delay_ms;
			float hr;
			//read float from message
			hr = float_from_wire(Data); // b/m
			float bps = hr / 60.0; /* b/m * m/s = b/s */
			float spb = 1.0/bps;
			float ms_p_b = 1000*spb;
			pulse_delay_ms = ms_p_b;
			//convert bpm as float to ms of delay as int
			//do whatever end up with ms of delay per beat as int
			//send to pulse unit
			pulse_set_delay(pulse_delay_ms);
			
			break;
		}
	}
}

