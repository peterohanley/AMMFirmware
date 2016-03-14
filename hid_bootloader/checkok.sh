#!/bin/sh
#how to check that a prospective program image is safe for uploading
oldhex=$1
newhex=$2

echo "$oldhex"
echo "$newhex"
#DOESN'T WORK YET
read -p "Please connect the avrispmkii programmer." yn
#use avrdude to dump flash
avrdude -p atmega32u4 -P usb -c avrispmkii -U flash:r:checkok_dump.bin:r

read -p "Please disconnect the avrispmkii programmer." yn
#bootload new image
#needs the new version that correctly triggers the bootloader from the applicaiton
#HostLoaderApp/hid_bootloader_cli -mmcu=atmega32u4 -v -w "$newhex"
#bootload old image over new image

#HostLoaderApp/hid_bootloader_cli -mmcu=atmega32u4 -v -w "$oldhex"

read -p "Please connect the avrispmkii programmer." yn
#check byte-for-byte compatibility with originally written image

avrdude -p atmega32u4 -P usb -c avrispmkii -U flash:r:checkok_dump_2.bin:r
#this requires .hex versions of old and new code, as well as knowledge that the
#	old code is on the device

diff checkok_dump.bin checkok_dump_2.bin
