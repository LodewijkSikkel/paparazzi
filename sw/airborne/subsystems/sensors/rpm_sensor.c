/*
 * Copyright (C) Bart Slinger
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/rpm_sensor/rpm_sensor.c"
 * @author Bart Slinger
 * Measure the ppm signal of the RPM sensor
 */

// Include own header
#include "subsystems/sensors/rpm_sensor.h"

#include "subsystems/datalink/telemetry.h"

struct RpmSensor rpm_sensor;

static void send_freq(struct transport_tx *trans, struct link_device *dev)
{
  // temporary placeholder
  float temp[1] = {rpm_sensor.motor_frequency};

  pprz_msg_send_MOTOR_FREQ(trans, dev, AC_ID, 1, temp);
}

void rpm_sensor_init(void)
{
  register_periodic_telemetry(DefaultPeriodic, "MOTOR_FREQ", send_freq);
  rpm_sensor_arch_init();
}

void rpm_sensor_process_pulse(uint16_t cnt, uint8_t overflow_cnt)
{
  (void) overflow_cnt;
  uint16_t diff = cnt - rpm_sensor.previous_cnt;

  if ((cnt > rpm_sensor.previous_cnt && overflow_cnt > 0) || (overflow_cnt > 1)) {
    rpm_sensor.motor_frequency = 0.0f;
  } else {
    rpm_sensor.motor_frequency = 281250.0/diff/12.0; // 72Mhz CPU with a prescaler of 256
  }

  /* Remember count */
  rpm_sensor.previous_cnt = cnt;
}

