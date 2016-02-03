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
 *  \brief Application Configuration Header File
 *
 *  This is a header file which is be used to configure some of
 *  the application's compile time options, as an alternative to
 *  specifying the compile time constants supplied through a
 *  makefile or build system.
 *
 *  For information on what each token does, refer to the
 *  \ref Sec_Options section of the application documentation.
 */

#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

	#define MAGIC_BOOT_KEY 0xDC42ACCA
	#define BOOTLOADER_SEC_SIZE_BYTES 4096
	#define FLASH_SIZE_BYTES 0x8000
	#define BOOTLOADER_START_ADDRESS  ((FLASH_SIZE_BYTES - BOOTLOADER_SEC_SIZE_BYTES) >> 1)
	#define START_BOOTLOADER_REPORT_ID 0xff

	#define DEVICE_SERIALNUMBER_BYTE_LENGTH 8
	#define GET_SERIAL_NUMBER_REPORT_ID 0xfe


	#define GENERIC_REPORT_SIZE 8
	//#define STRING_DESC_REPORT_HACK_SIZE 34

	#define REPORT_MAP_STRING_ID 1
#define REPORT_MAP_STRING_SIZE 0xfe
//#define REPORT_MAP_STRING_SIZE 64
	#define STRING_DESC_HACK_REPORT_ID 2

	#define TIMESTAMP_OFFSET_FR_ID 4
	#define TIMESTAMP_FR_SIZE 0x08

	#define WIRE_CONTACT_REPORT_ID 5
	#define WIRE_CONTACT_REPORT_SIZE 12

	#define SET_SENS_THRESH_REPORT_ID 6
	#define SET_SENS_THRESH_REPORT_SIZE 16

	#define GET_SENSOR_EVENT_REPORT_ID 7
	#define GET_SENSOR_EVENT_REPORT_SIZE 13


	#define RESPIRATORY_RATE_REPORT_ID 8
	#define TIDAL_VOLUME_REPORT_ID 9
	#define LUNG_VOLUME_TOTAL_REPORT_ID 10
	#define LUNG_VOLUME_L_REPORT_ID 11
	#define LUNG_VOLUME_R_REPORT_ID 12
	#define HEART_RATE_REPORT_ID 13
	#define ART_REPORT_ID 14

	#define NO_DATA_REPORT_ID 15
	#define NO_DATA_REPORT_SIZE 4

	#define RFID_TAG_PASSTHROUGH_REPORT_ID 16
	#define RFID_TAG_PASSTHROUGH_REPORT_SIZE 64
	#define RFID_TAG_SCAN_COMMAND_REPORT_ID 17

	#define BIO_EVENT_REPORT_ID 18
#define BIO_EVENT_REPORT_SIZE 255
//#define BIO_EVENT_REPORT_SIZE 64

	#define PROX_REPORT_ID 19
#define PROX_REPORT_SIZE 255
//#define PROX_REPORT_SIZE 64

	#define DEVICE_NAME_REPORT_ID 2
#define DEVICE_NAME_REPORT_SIZE 255
//#define DEVICE_NAME_REPORT_SIZE 64

//above is the required definitions. Now the device selection

/* features:
	rfid
	pulse
	running variance
	ADC readings
*/


//#define RV_STM_COUNT 4
//#define RV_STM_ADC_NUMS {1,4,6,7}

#define RV_IS_IV_ARM 0

//*for iv infusion
#define RV_STM_COUNT 1
#define RV_STM_ADC_NUMS {0}
//*/
//controls whether sensor data is sent back
#define SEND_ADC_DATA 0

//send flow sensor debug messages?
//#define SEND_BLIP

//*
#define BIO_REPORT_TABLE(_) \
	_(ACT,18,"s")\
	_(PROX,19,"s")

#define BIO_AS_REPORT_STRING(name, num, suffix) #name "=" #num suffix ","
//*/
		
	#define DEFINE_PSTRING(var,str) const struct {unsigned char len; char content[sizeof(str)];} PROGMEM (var) = {sizeof(str)-1, (str)}
		

	#define UNUSED(x) (void)x;

#endif
