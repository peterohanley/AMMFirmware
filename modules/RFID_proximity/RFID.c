#include "RFID.h"

typedef struct {
	char tag;
	char len;
	char label;
	char qi_value;
	char freq[3];
	char epc_len;
	uint16_t reserved;
	char rfid_tag[12];
} rfid_read_response;
//above might not always be the case

typedef struct {
	char tag;
	char len;
	char data[0];
} rfid_basic_response;

void rfid_parser_clearbuffers(void)
{
	for (int i = 0; i<64; i++) {
		(*inprog_message)[i] = (*usable_message)[i] =0;
	}
}

void rfid_sendcommand_readtags(void)
{
	Serial_SendByte(0x43);
	Serial_SendByte(0x03);
	Serial_SendByte(0x01);
}

char msgbuf_1[64];
char msgbuf_2[64];
char (*usable_message)[64] = &msgbuf_1;
char (*inprog_message)[64] = &msgbuf_2;
bool rfid_usable_to_send;
int curmsg_firstempty = 0;
ms_time_t last_rfid_read;
ms_time_t last_buzzer_enable;

//*
//messages RFID needs to send on PROX
#define RFID_MESSAGE_TABLE(_) _(bvm, BVM)\
	_(vent,VENT)\
	_(etomidate,ETOMIDATE)\
	_(succs,SUCCS)\
	_(morphine,MORPHINE)\
	_(fentanyl,FENTANYL)\
	_(ketamine,KETAMINE)\
	_(antibiotics,ANTIBIOTICS)\
	_(propofol,PROPOFOL)\
	_(lidocane,LIDOCANE)\
	_(roc,ROC)\
	_(fluids,FLUIDS)\
	_(ecg,ECG)\
	_(bp_cuff,BP_CUFF)\
	_(pulse_ox,PULSE_OX)\
	_(temp_probe,TEMP_PROBE)\
	_(o2,O2)
//*/
#define AS_PSTRING_DEF(name,str) DEFINE_PSTRING(name##_str,#str);
RFID_MESSAGE_TABLE(AS_PSTRING_DEF);
#define TAG_LEN 12
#define DEBOUNCE_MS 5000
typedef struct {
	unsigned char tag[TAG_LEN];
	//there is a string just past it of that length.
	const unsigned char* msglen;
	const char* msg;
	ms_time_t last_send_time;
} tag_magic_t;

//TODO put this table in PROGMEM. be sure to change SEND_PROX_UGLY_HACK and classify_tag to read from appropriate memories!
#define TAG_MAGIC_COUNT (sizeof(tags)/sizeof(tags[0]))
tag_magic_t tags[] = {
	{
		.tag= {0xe2, 0x80, 0x68, 0x10, 0x00, 0x00, 0x00, 0x39, 0x60, 0x11, 0x97, 0x46},
		.msglen = &bvm_str.len,
		.msg = bvm_str.content,
	},{
		.tag= {0xad, 0x46, 0x06, 0x02, 0x46, 0x60, 0xaf, 0x79, 0x60, 0x00, 0x00, 0x68},
		.msglen = &etomidate_str.len,
		.msg = etomidate_str.content,
	},{
		.tag= {0xAD, 0x46, 0x06, 0x02, 0x46, 0x60, 0x93, 0x78, 0x60, 0x00, 0x00, 0x64},
		.msglen = &succs_str.len,
		.msg = succs_str.content,
	},{
		.tag= {0xAD, 0x46, 0x06, 0x02, 0x46, 0x60, 0x91, 0x75, 0x60, 0x00, 0x00, 0x65},
		.msglen = &succs_str.len,
		.msg = succs_str.content,
	},{
		.tag= {0xE2, 0x80, 0x68, 0x10, 0x00, 0x00, 0x00, 0x39, 0x14, 0x8C, 0x72, 0xE0},
		.msglen = &morphine_str.len,
		.msg = morphine_str.content,
	},{
		.tag= {0xad, 0x46, 0x06, 0x02, 0x46, 0x60, 0x9d, 0x73, 0x61, 0x00, 0x00, 0x66},
		.msglen = &morphine_str.len,
		.msg = morphine_str.content,
	},{
		.tag= {0xAD, 0x46, 0x06, 0x02, 0x46, 0x60, 0xA5, 0x78, 0x60, 0x00, 0x00, 0x67},
		.msglen = &fentanyl_str.len,
		.msg = fentanyl_str.content,
	},{
		.tag= {0xE2, 0x80, 0x68, 0x10, 0x00, 0x00, 0x00, 0x39, 0x14, 0x8C, 0x72, 0xE3},
		.msglen = &fluids_str.len,
		.msg = fluids_str.content,
	},{
		.tag= {0xAD, 0x46, 0x06, 0x02, 0x46, 0x60, 0xE1, 0x78, 0x60, 0x00, 0x00, 0x6D},
		.msglen = &propofol_str.len,
		.msg = propofol_str.content,
	},{
		.tag= {0xAD, 0x46, 0x06, 0x02, 0x46, 0x60, 0xD5, 0x7B, 0x60, 0x00, 0x00, 0x6C},
		.msglen = &roc_str.len,
		.msg = roc_str.content,
	},{
		.tag= {0xAD, 0x46, 0x06, 0x02, 0x46, 0x60, 0xCD, 0x75, 0x61, 0x00, 0x00, 0x6B},
		.msglen = &ketamine_str.len,
		.msg = ketamine_str.content,
	},{
		.tag= {0xAD, 0x46, 0x06, 0x02, 0x46, 0x60, 0xC3, 0x76, 0x61, 0x00, 0x00, 0x6A},
		.msglen = &o2_str.len,
		.msg = o2_str.content,
	},{
		.tag= {0xE2, 0x80, 0x68, 0x10, 0x00, 0x00, 0x00, 0x39, 0x14, 0x8C, 0x73, 0x2A},
		.msglen = &ecg_str.len,
		.msg = ecg_str.content,
	},{
		.tag= {0xE2, 0x80, 0x68, 0x10, 0x00, 0x00, 0x00, 0x39, 0x14, 0x8C, 0x73, 0x2D},
		.msglen = &pulse_ox_str.len,
		.msg = pulse_ox_str.content,
	},{
		.tag= {0xAD, 0x46, 0x06, 0x02, 0x46, 0x60, 0xE9, 0x7A, 0x60, 0x00, 0x00, 0x6E},
		.msglen = &bp_cuff_str.len,
		.msg = bp_cuff_str.content,
	},{
		.tag= {0xAD, 0x46, 0x06, 0x02, 0x46, 0x60, 0xf5, 0x78, 0x60, 0x00, 0x00, 0x6f},
		.msglen = &vent_str.len,
		.msg = vent_str.content,
	},
	
};



bool classify_tag(const unsigned char* tagbuf, unsigned char* out_len, const char** out_str)
{
	int c;
	ms_time_t now = millis();
	for (unsigned int i = 0; i < TAG_MAGIC_COUNT;i++) {
		c = memcmp((void*)tagbuf,(void*) tags[i].tag,TAG_LEN);
		if (!c) {
			if ((now-tags[i].last_send_time) > DEBOUNCE_MS) {
				tags[i].last_send_time = now;
				*out_len = pgm_read_byte(tags[i].msglen);
				*out_str = tags[i].msg;
				return 1; // str not null, true
			} else {
				tags[i].last_send_time = now;
				return 1;
			}
		}
	}
	return 0;
}

#define RFID_MS_BETWEEN_READS 250
#define RFID_BUZZER_TIME 500
void rfid_init(void)
{
	last_rfid_read = millis();
}
void rfid_task(void)
{
	ms_time_t now = millis();
	if ((now - last_rfid_read) > RFID_MS_BETWEEN_READS) {
		last_rfid_read = now;
		rfid_sendcommand_readtags();
	}
	if ((now - last_buzzer_enable) > RFID_BUZZER_TIME) {
		PORTB &= ~(1<<PB7);
	}
	
}

void rfid_enable_buzzer(void)
{
	last_buzzer_enable = millis();
	PORTB |= (1<<PB7);
}
	
bool try_parse_message(void)
{
	while (Serial_IsCharReceived()) {
		(*inprog_message)[curmsg_firstempty] = Serial_ReceiveByte();
		//Serial_SendByte((*inprog_message)[curmsg_firstempty]);
		
		/* dumb idea for resync - we only care about one kind of message */
		//*
		curmsg_firstempty++;
		
		if (curmsg_firstempty == 1) {
			if ((*inprog_message)[0] != 0x44 && (*inprog_message)[0] != 0x19
				&& (*inprog_message)[0] != 0x1b) {
				curmsg_firstempty = 0;
			}
		} else if (curmsg_firstempty > 1) {
			char len = (*inprog_message)[1];
			if (curmsg_firstempty >= len || curmsg_firstempty >= 64) {
				char (*t)[64] = usable_message;
				usable_message = inprog_message;
				inprog_message = t;
				//for (int i = 0; i<64; i++) {
				//	(*inprog_message)[i]=0;
				//}
				curmsg_firstempty = 0;
				rfid_usable_to_send = 1;
				//we have a fully parsed message, and we have reset current
				return 1;
			}
		}
		//*/
	}
	return 0;
}