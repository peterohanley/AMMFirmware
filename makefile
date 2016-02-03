#
#             LUFA Library
#     Copyright (C) Dean Camera, 2015.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Run "make help" for target help.

MCU          = atmega32u4
ARCH         = AVR8
BOARD        = LEONARDO
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = GenericHID
SRC          = $(TARGET).c Descriptors.c AirwaySensors.c Timer.c RFID.c EscharotomyArm.c FlowSensor.c RunningVariance.c $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)
LUFA_PATH    = ../../../lufa/LUFA
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -IConfig/ -Werror -Wshadow -Wextra -Wall
LD_FLAGS     =


AVRDUDE_PORT = usb
AVRDUDE_PROGRAMMER = avrispmkII
AVRDUDE_FLAGS = -vv

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk
