
PAPARAZZI_SRC = ../../..
DRONE = $(PAPARAZZI_SRC)/sw/tools/parrot/ardrone2.py

SUB_DIR = bl

HOST = 192.168.1.1

 # Program the device and start it.
upload_driver:
	$(Q)$(DRONE) --host=$(HOST) insmod cdc-acm.ko
	$(Q)$(DRONE) --host=$(HOST) status

.PHONY: upload driver
