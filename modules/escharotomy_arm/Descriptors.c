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
 *  USB Device Descriptors, for library use when in USB device mode. Descriptors are special
 *  computer-readable structures which the host requests upon device enumeration, to determine
 *  the device's capabilities and functions.
 */

#include "Descriptors.h"

#define STANDARD_INPUT_REPORT(rid,usg,byts) HID_RI_REPORT_ID(8,rid),\
	HID_RI_USAGE(8, usg),\
	HID_RI_LOGICAL_MINIMUM(8,0x00),\
	HID_RI_LOGICAL_MAXIMUM(16,0x00FF),\
	HID_RI_REPORT_SIZE(8, 8),\
	HID_RI_REPORT_COUNT(8, byts),\
	HID_RI_INPUT(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE)

#define STANDARD_OUTPUT_REPORT(rid,usg,byts) HID_RI_REPORT_ID(8,rid),\
	HID_RI_USAGE(8, usg),\
    HID_RI_LOGICAL_MINIMUM(8, 0x00),\
    HID_RI_LOGICAL_MAXIMUM(16, 0x00FF),\
    HID_RI_REPORT_SIZE(8, 0x08),\
    HID_RI_REPORT_COUNT(8, byts),\
    HID_RI_OUTPUT(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE)

#define STANDARD_FEATURE_REPORT(rid,usg,byts) HID_RI_REPORT_ID(8,rid),\
	HID_RI_USAGE(8, usg),\
	HID_RI_LOGICAL_MINIMUM(8, 0x00),\
    HID_RI_LOGICAL_MAXIMUM(16, 0x00FF),\
    HID_RI_REPORT_SIZE(8, 0x08),\
    HID_RI_REPORT_COUNT(8, byts),\
    HID_RI_FEATURE(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE)

#define STANDARD_INANDOUT_REPORT(r,u,b) STANDARD_INPUT_REPORT(r,u,b),\
			STANDARD_OUTPUT_REPORT(r,u,b)

/** HID class report descriptor. This is a special descriptor constructed with values from the
 *  USBIF HID class specification to describe the reports and capabilities of the HID device. This
 *  descriptor is parsed by the host and its contents used to determine what data (and in what encoding)
 *  the device will send, and what it may be sent back from the host. Refer to the HID specification for
 *  more details on HID report descriptors.
 */
const USB_Descriptor_HIDReport_Datatype_t PROGMEM GenericReport[] =
{
	/* Use the HID class driver's standard Vendor HID report.
	 *  Vendor Usage Page: 0
	 *  Vendor Collection Usage: 1
	 *  Vendor Report IN Usage: 2
	 *  Vendor Report OUT Usage: 3
	 *  Vendor Report Size: GENERIC_REPORT_SIZE
	 */
	//HID_DESCRIPTOR_VENDOR(0x00, 0x01, 0x02, 0x03, GENERIC_REPORT_SIZE)

	HID_RI_USAGE_PAGE(16, 0xFF00), /* Vendor Page 0 */
	HID_RI_USAGE(8, 0x01), /* Vendor Usage 1 */
	HID_RI_COLLECTION(8, 0x01), /* Vendor Usage 1 */
#if 0
		HID_RI_REPORT_ID(8, 0x01), /* report id */
	    HID_RI_USAGE(8, 0x02), /* Vendor Usage 2 */
	    HID_RI_LOGICAL_MINIMUM(8, 0x00),
	    HID_RI_LOGICAL_MAXIMUM(16, 0x00FF),
	    HID_RI_REPORT_SIZE(8, 0x08),
	    HID_RI_REPORT_COUNT(8, GENERIC_REPORT_SIZE),
	    HID_RI_INPUT(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
	    HID_RI_USAGE(8, 0x03), /* Vendor Usage 3 */
	    HID_RI_LOGICAL_MINIMUM(8, 0x00),
	    HID_RI_LOGICAL_MAXIMUM(16, 0x00FF),
	    HID_RI_REPORT_SIZE(8, 0x08),
	    HID_RI_REPORT_COUNT(8, GENERIC_REPORT_SIZE),
	    HID_RI_OUTPUT(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE | HID_IOF_NON_VOLATILE),
		HID_RI_USAGE(8, 0x04),
		HID_RI_LOGICAL_MINIMUM(8, 0x00),
	    HID_RI_LOGICAL_MAXIMUM(16, 0x00FF),
	    HID_RI_REPORT_SIZE(8, 0x08),
	    HID_RI_REPORT_COUNT(8, GENERIC_REPORT_SIZE),
	    HID_RI_FEATURE(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
#endif
	STANDARD_FEATURE_REPORT(REPORT_MAP_STRING_ID,0x02,REPORT_MAP_STRING_SIZE),
		
#if 0
		HID_RI_REPORT_ID(8, 0x02), /* report id */
		HID_RI_USAGE(8, 0x05),
		HID_RI_LOGICAL_MINIMUM(8, 0x00),
	    HID_RI_LOGICAL_MAXIMUM(16, 0x00FF),
	    HID_RI_REPORT_SIZE(8, 0x08), /* a byte */
	    HID_RI_REPORT_COUNT(8, STRING_DESC_REPORT_HACK_SIZE),
	    HID_RI_FEATURE(8, HID_IOF_CONSTANT | HID_IOF_ARRAY | HID_IOF_ABSOLUTE),
#endif
	
	STANDARD_FEATURE_REPORT(DEVICE_NAME_REPORT_ID, 0x02, DEVICE_NAME_REPORT_SIZE),
		
		HID_RI_REPORT_ID(8, 0x03), /* report id */
		HID_RI_USAGE(8,0x05), /* Vendor usage 4 */
		HID_RI_LOGICAL_MINIMUM(8, 0x00),
	    HID_RI_LOGICAL_MAXIMUM(16, 0x00FF), /* TODO use the real values if this wasn't the problem. */
		HID_RI_REPORT_SIZE(8, 0x8), /* a word */
		HID_RI_REPORT_COUNT(8, RV_STM_COUNT*6), /* A0 - A5 */
		HID_RI_INPUT(8, HID_IOF_DATA | HID_IOF_ARRAY | HID_IOF_ABSOLUTE),
		
		HID_RI_REPORT_ID(8, TIMESTAMP_OFFSET_FR_ID), /* report id */ /* set/get current time, ms */
		HID_RI_USAGE(8,0x06), /* Vendor usage 4 */
		HID_RI_LOGICAL_MINIMUM(8, 0x00),
	    HID_RI_LOGICAL_MAXIMUM(16, 0x00FF), 
		HID_RI_REPORT_SIZE(8, 0x8), /* a byte */
		HID_RI_REPORT_COUNT(8, TIMESTAMP_FR_SIZE), /* 8 bytes, one uint64_t */
		HID_RI_FEATURE(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
		
		/* WIRE_CONTACT_REPORT_ID */
		HID_RI_REPORT_ID(8, WIRE_CONTACT_REPORT_ID), 
		HID_RI_USAGE(8,0x07), /* Vendor usage 4 */
		HID_RI_LOGICAL_MINIMUM(8, 0x00),
	    HID_RI_LOGICAL_MAXIMUM(16, 0x00FF), 
		HID_RI_REPORT_SIZE(8, 0x8), /* a byte */
		HID_RI_REPORT_COUNT(8, WIRE_CONTACT_REPORT_SIZE),
		HID_RI_INPUT(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
		
		/* SET_SENS_THRESH_REPORT_ID */
		HID_RI_REPORT_ID(8, SET_SENS_THRESH_REPORT_ID),
		HID_RI_USAGE(8,0x08), 
		HID_RI_LOGICAL_MINIMUM(8, 0x00),
	    HID_RI_LOGICAL_MAXIMUM(16, 0x00FF), 
		HID_RI_REPORT_SIZE(8, 0x8), /* a byte */
		HID_RI_REPORT_COUNT(8, SET_SENS_THRESH_REPORT_SIZE),
		HID_RI_FEATURE(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
		
		/* GET_SENSOR_EVENT_REPORT_ID */
		HID_RI_REPORT_ID(8, GET_SENSOR_EVENT_REPORT_ID),
		HID_RI_USAGE(8,0x09), 
		HID_RI_LOGICAL_MINIMUM(8, 0x00),
	    HID_RI_LOGICAL_MAXIMUM(16, 0x00FF), 
		HID_RI_REPORT_SIZE(8, 0x8), /* a byte */
		HID_RI_REPORT_COUNT(8, GET_SENSOR_EVENT_REPORT_SIZE),
		HID_RI_INPUT(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
		
		HID_RI_REPORT_ID(8,START_BOOTLOADER_REPORT_ID),
		HID_RI_USAGE(8, 0x0a), // FIXME this probably has a proper value rather than "autoincrement"
		HID_RI_LOGICAL_MINIMUM(8,0x00),
		HID_RI_LOGICAL_MAXIMUM(16,0x00FF),
		HID_RI_REPORT_SIZE(8, 0x08),
		HID_RI_REPORT_COUNT(8, DEVICE_SERIALNUMBER_BYTE_LENGTH),
		HID_RI_FEATURE(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
		
		HID_RI_REPORT_ID(8,GET_SERIAL_NUMBER_REPORT_ID),
		HID_RI_USAGE(8, 0x0b), // FIXME this probably has a proper value rather than "autoincrement"
		HID_RI_LOGICAL_MINIMUM(8,0x00),
		HID_RI_LOGICAL_MAXIMUM(16,0x00FF),
		HID_RI_REPORT_SIZE(8, 0x08),
		HID_RI_REPORT_COUNT(8, DEVICE_SERIALNUMBER_BYTE_LENGTH),
		HID_RI_FEATURE(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
		
		//STANDARD_INPUT_REPORT(
		STANDARD_INANDOUT_REPORT( RESPIRATORY_RATE_REPORT_ID, 0x0c, 4),
		STANDARD_INANDOUT_REPORT( TIDAL_VOLUME_REPORT_ID, 0x0c, 4),
		STANDARD_INANDOUT_REPORT( LUNG_VOLUME_TOTAL_REPORT_ID, 0x0c, 4),
		STANDARD_INANDOUT_REPORT( LUNG_VOLUME_L_REPORT_ID, 0x0c, 4),
		STANDARD_INANDOUT_REPORT( LUNG_VOLUME_R_REPORT_ID, 0x0c, 4),
		STANDARD_INANDOUT_REPORT( HEART_RATE_REPORT_ID, 0x0c, 4),
		STANDARD_INANDOUT_REPORT( ART_REPORT_ID, 0x0c, 4),
		
		STANDARD_FEATURE_REPORT(NO_DATA_REPORT_ID, 0, NO_DATA_REPORT_SIZE),
		STANDARD_INPUT_REPORT(RFID_TAG_PASSTHROUGH_REPORT_ID, 0x0d, 64),
		STANDARD_OUTPUT_REPORT(RFID_TAG_SCAN_COMMAND_REPORT_ID, 0x0d, 1),
		
		//something about this report messes with SimpleHIDWrite on windows
		STANDARD_INANDOUT_REPORT(BIO_EVENT_REPORT_ID, 0x0e, BIO_EVENT_REPORT_SIZE),
		STANDARD_INANDOUT_REPORT(PROX_REPORT_ID, 0x0f, PROX_REPORT_SIZE),
	HID_RI_END_COLLECTION(0),
};

/** Device descriptor structure. This descriptor, located in FLASH memory, describes the overall
 *  device characteristics, including the supported USB version, control endpoint size and the
 *  number of device configurations. The descriptor is read out by the USB host when the enumeration
 *  process begins.
 */
const USB_Descriptor_Device_t PROGMEM DeviceDescriptor =
{
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},

	.USBSpecification       = VERSION_BCD(1,1,0),
	.Class                  = USB_CSCP_NoDeviceClass,
	.SubClass               = USB_CSCP_NoDeviceSubclass,
	.Protocol               = USB_CSCP_NoDeviceProtocol,

	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,

	.VendorID               = 0x03EB,
	.ProductID              = 0x2040,
	.ReleaseNumber          = VERSION_BCD(0,0,1),

	.ManufacturerStrIndex   = STRING_ID_Manufacturer,
	.ProductStrIndex        = STRING_ID_Product,
	.SerialNumStrIndex      = NO_DESCRIPTOR,

	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

/** Configuration descriptor structure. This descriptor, located in FLASH memory, describes the usage
 *  of the device in one of its supported configurations, including information about any device interfaces
 *  and endpoints. The descriptor is read out by the USB host during the enumeration process when selecting
 *  a configuration so that the host may correctly communicate with the USB device.
 */
const USB_Descriptor_Configuration_t PROGMEM ConfigurationDescriptor =
{
	.Config =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},

			.TotalConfigurationSize = sizeof(USB_Descriptor_Configuration_t),
			.TotalInterfaces        = 1,

			.ConfigurationNumber    = 1,
			.ConfigurationStrIndex  = NO_DESCRIPTOR,

			.ConfigAttributes       = (USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED),

			.MaxPowerConsumption    = USB_CONFIG_POWER_MA(100)
		},

	.HID_Interface =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = INTERFACE_ID_GenericHID,
			.AlternateSetting       = 0x00,

			.TotalEndpoints         = 1,

			.Class                  = HID_CSCP_HIDClass,
			.SubClass               = HID_CSCP_NonBootSubclass,
			.Protocol               = HID_CSCP_NonBootProtocol,

			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

	.HID_GenericHID =
		{
			.Header                 = {.Size = sizeof(USB_HID_Descriptor_HID_t), .Type = HID_DTYPE_HID},

			.HIDSpec                = VERSION_BCD(1,1,1),
			.CountryCode            = 0x00,
			.TotalReportDescriptors = 1,
			.HIDReportType          = HID_DTYPE_Report,
			.HIDReportLength        = sizeof(GenericReport)
		},

	.HID_ReportINEndpoint =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},

			.EndpointAddress        = GENERIC_IN_EPADDR,
			.Attributes             = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = GENERIC_EPSIZE,
			.PollingIntervalMS      = 0x05
		},
};

/** Language descriptor structure. This descriptor, located in FLASH memory, is returned when the host requests
 *  the string descriptor with index 0 (the first index). It is actually an array of 16-bit integers, which indicate
 *  via the language ID table available at USB.org what languages the device supports for its string descriptors.
 */
const USB_Descriptor_String_t PROGMEM LanguageString = USB_STRING_DESCRIPTOR_ARRAY(LANGUAGE_ID_ENG);

/** Manufacturer descriptor string. This is a Unicode string containing the manufacturer's details in human readable
 *  form, and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
const USB_Descriptor_String_t PROGMEM ManufacturerString = USB_STRING_DESCRIPTOR(L"Dean Camera");

/** Product descriptor string. This is a Unicode string containing the product's details in human readable form,
 *  and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
const USB_Descriptor_String_t PROGMEM ProductString = USB_STRING_DESCRIPTOR(L"LUFA Generic HID Demo");

/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 */
uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint8_t wIndex,
                                    const void** const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

	const void* Address = NULL;
	uint16_t    Size    = NO_DESCRIPTOR;
	UNUSED(wIndex);
	switch (DescriptorType)
	{
		case DTYPE_Device:
			Address = &DeviceDescriptor;
			Size    = sizeof(USB_Descriptor_Device_t);
			break;
		case DTYPE_Configuration:
			Address = &ConfigurationDescriptor;
			Size    = sizeof(USB_Descriptor_Configuration_t);
			break;
		case DTYPE_String:
			switch (DescriptorNumber)
			{
				case STRING_ID_Language:
					Address = &LanguageString;
					Size    = pgm_read_byte(&LanguageString.Header.Size);
					break;
				case STRING_ID_Manufacturer:
					Address = &ManufacturerString;
					Size    = pgm_read_byte(&ManufacturerString.Header.Size);
					break;
				case STRING_ID_Product:
					Address = &ProductString;
					Size    = pgm_read_byte(&ProductString.Header.Size);
					break;
			}

			break;
		case HID_DTYPE_HID:
			Address = &ConfigurationDescriptor.HID_GenericHID;
			Size    = sizeof(USB_HID_Descriptor_HID_t);
			break;
		case HID_DTYPE_Report:
			Address = &GenericReport;
			Size    = sizeof(GenericReport);
			break;
	}

	*DescriptorAddress = Address;
	return Size;
}

