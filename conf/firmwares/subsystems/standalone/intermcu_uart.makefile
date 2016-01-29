# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2015 Lodewijk Sikkel <l.n.c.sikkel@tudelft.nl>
#
# InterMCU type standalone
#

ifeq ($(TARGET),ekf)
	INTERMCU_PORT ?= UART2
	INTERMCU_PORT_LOWER = $(shell echo $(INTERMCU_PORT) | tr A-Z a-z)
	ekf.CFLAGS += -DINTERMCU_EKF
	ekf.CFLAGS += -DINTERMCU_LINK=$(INTERMCU_PORT_LOWER) -DUSE_$(INTERMCU_PORT) -D$(INTERMCU_PORT)_BAUD=B230400
	ekf.CFLAGS += -DDOWNLINK
	ekf.srcs += $(PAPARAZZI_HOME)/var/share/pprzlink/src/pprz_transport.c
	ekf.srcs += subsystems/intermcu/intermcu_standalone.c
else
	INTERMCU_PORT ?= UART3
	INTERMCU_PORT_LOWER = $(shell echo $(INTERMCU_PORT) | tr A-Z a-z)
	ap.CFLAGS += -DINTERMCU_AP
	ap.CFLAGS += -DINTERMCU_LINK=$(INTERMCU_PORT_LOWER) -DUSE_$(INTERMCU_PORT) -D$(INTERMCU_PORT)_BAUD=B230400
	ap.CFLAGS += -DDOWNLINK
	ap.srcs += $(PAPARAZZI_HOME)/var/share/pprzlink/src/pprz_transport.c
	ap.srcs += subsystems/intermcu/intermcu_standalone.c
endif