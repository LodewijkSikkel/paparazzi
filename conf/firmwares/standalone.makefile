# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2015 Lodewijk Sikkel <l.n.c.sikkel@tudelft.nl>
#
# Standalone firmware
#

CFG_SHARED=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/shared

CFG_STANDALONE=$(PAPARAZZI_SRC)/conf/firmwares/subsystems/standalone

SRC_BOARD=boards/$(BOARD)
SRC_SUBSYSTEMS=subsystems
SRC_MODULES=modules

SRC_ARCH=arch/$(ARCH)

#
# Common
#
PERIODIC_FREQUENCY ?= 512

COMMON_STANDALONE_CFLAGS  = -I$(SRC_BOARD) 
COMMON_STANDALONE_CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
COMMON_STANDALONE_CFLAGS += -DPERIPHERALS_AUTO_INIT

COMMON_STANDALONE_SRCS    = mcu.c 
COMMON_STANDALONE_SRCS   += $(SRC_ARCH)/mcu_arch.c
ifneq ($(SYS_TIME_LED),none)
  COMMON_STANDALONE_CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif
COMMON_STANDALONE_CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)

# 
# Systime
#
COMMON_STANDALONE_SRCS   += mcu_periph/sys_time.c 
COMMON_STANDALONE_SRCS   += $(SRC_ARCH)/mcu_periph/sys_time_arch.c
ifeq ($(ARCH), linux)
# seems that we need to link agains librt for glibc < 2.17
$(TARGET).LDFLAGS += -lrt
endif

# 
# Leds
#
COMMON_STANDALONE_CFLAGS += -DUSE_LED
ifeq ($(ARCH), lpc21)
COMMON_STANDALONE_SRCS += $(SRC_ARCH)/armVIC.c
else ifeq ($(ARCH), stm32)
COMMON_STANDALONE_SRCS += $(SRC_ARCH)/led_hw.c
COMMON_STANDALONE_SRCS += $(SRC_ARCH)/mcu_periph/gpio_arch.c
endif

#
# Telemetry
#
ifdef INTERMCU_STANDALONE_TEST
ifndef MODEM_PORT
MODEM_PORT ?= UART2
endif

ifndef MODEM_BAUD
MODEM_BAUD ?= B57600
endif

COMMON_MODEM_PORT_LOWER=$(shell echo $(MODEM_PORT) | tr A-Z a-z)
COMMON_STANDALONE_CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=pprz_tp -DDATALINK=PPRZ
COMMON_STANDALONE_CFLAGS += -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
COMMON_STANDALONE_CFLAGS += -DPPRZ_UART=$(COMMON_MODEM_PORT_LOWER)
COMMON_STANDALONE_CFLAGS += -DDOWNLINK_DEVICE=$(COMMON_MODEM_PORT_LOWER)
COMMON_STANDALONE_SRCS   += subsystems/datalink/downlink.c $(PAPARAZZI_HOME)/var/share/pprzlink/src/pprz_transport.c
endif

#
# Extended Kalman filter
#
ifeq ($(BOARD), lisa_m)
ifeq ($(BOARD_VERSION), 2.0)
LED_DEFINES = -DLED_BLUE=3 -DLED_RED=4 -DLED_GREEN=5
endif
endif
ifeq ($(BOARD), navstik)
LED_DEFINES = -DLED_RED=1 -DLED_GREEN=2
endif
ifeq ($(BOARD), cc3d)
LED_DEFINES = -DLED_BLUE=1
endif
ifeq ($(BOARD), naze32)
LED_DEFINES = -DLED_RED=1 -DLED_GREEN=2
endif
LED_DEFINES ?= -DLED_RED=2 -DLED_GREEN=3

ekf.ARCHDIR = $(ARCH)
ekf.CFLAGS  = $(COMMON_STANDALONE_CFLAGS) $(LED_DEFINES)
ekf.srcs    = $(COMMON_STANDALONE_SRCS)
ekf.srcs   += standalone/main_ekf.c

include $(CFG_SHARED)/uart.makefile

include $(CFG_STANDALONE)/intermcu_uart.makefile