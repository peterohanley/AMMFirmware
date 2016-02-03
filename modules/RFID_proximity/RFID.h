#ifndef _RFID_H_
#define _RFID_H_

	#include "Config/AppConfig.h"
	//#include <LUFA/Common/Common.h>
	#include <LUFA/Drivers/Peripheral/Serial.h>
	#include <string.h>
	#include "Timer.h"

	char (*usable_message)[64];
	char (*inprog_message)[64];
	bool rfid_usable_to_send;
	
	void rfid_init(void);
	void rfid_task(void);
	void rfid_parser_clearbuffers(void);
	void rfid_sendcommand_readtags(void);
	bool classify_tag(const unsigned char* tagbuf, unsigned char* out_len, const char** out_str);
	bool try_parse_message(void);
	void rfid_enable_buzzer(void);
#endif