#!/bin/sh

#bootstrap the bootloader onto an empty board
#for use when upgrading bootloader, not when writing a board, use a flahs image
#for that

#stage
#	empty

#stage
#	enable BOOTRST
avrdude -p atmega32u4 -P usb -c avrispmkii -U lfuse:w:0x5e:m -U hfuse:w:0x98:m -U efuse:w:0xcb:m

#	load new bootloader
avrdude -p atmega32u4 -P usb -c avrispmkii -U flash:w:BootloaderHID.hex

read -p "Please disconnect the avrispmkii programmer." yn

#stage
#	load new application through new bootloader

HostLoaderApp/hid_bootloader_cli -mmcu=atmega32u4 -v -w ../firmware/GenericHID.hex

read -p "Please reconnect the avrispmkii programmer." yn
#stage
#	disable BOOTRST

avrdude -p atmega32u4 -P usb -c avrispmkii -U lfuse:w:0x5e:m -U hfuse:w:0x99:m -U efuse:w:0xcb:m

