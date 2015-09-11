RPM_SENSOR_BLHELI_SRCS   =  $(SRC_SUBSYSTEMS)/sensors/rpm_sensor.c
RPM_SENSOR_BLHELI_SRCS   += $(SRC_ARCH)/subsystems/sensors/rpm_sensor_arch.c

$(TARGET).CFLAGS += -DUSE_RPM_SENSOR

$(TARGET).srcs += $(RPM_SENSOR_BLHELI_SRCS)