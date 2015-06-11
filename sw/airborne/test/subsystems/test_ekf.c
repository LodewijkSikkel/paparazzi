/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file test/subsystems/test_ekf.c
 * This program test the use of an onboard extended Kalman filter using 
 * a drag-force-enhanced model and accelerometer and gyro measurments.
 */

#include <inttypes.h>

#define ABI_C

#ifdef BOARD_CONFIG
#include BOARD_CONFIG
#endif
#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/i2c.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "subsystems/imu.h"
#include "subsystems/abi.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

static void ekf_time_update(void);
static void ekf_measurement_update(void);

static abi_event gyro_ev;
static abi_event accel_ev;
static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro);
static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel);

int main(void)
{
  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    main_event_task();
  }
  return 0;
} 

static inline void main_init(void)
{

  mcu_init();

  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);

  imu_init();

  mcu_int_enable();

  downlink_init();

  AbiBindMsgIMU_GYRO_INT32(0, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(0, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_INT32(0, &mag_ev, mag_cb);
}

static inline void main_periodic_task(void)
{
  RunOnceEvery(100, {
    led_toggle();
    DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
  });

  if (sys_time.nb_sec > 1) { 
  	imu_periodic(); 
  }
  
  RunOnceEvery(10, { LED_PERIODIC();});
}

static inline void main_event_task(void)
{
  mcu_event();
  ImuEvent();

  /* TODO: The gyro measurements (scaled) are used as input to the 
   * propagation model and should therefore be passed to the 
   * appropriate function (ekf_time_update()).
   */
  ekf_time_update();

  /* TODO: Upon receiving the IMU event the EKF measurement correction 
   * can be performed using the accelerometer measurements (scaled).
   */ 
  ekf_measurement_update();
}

static void ekf_time_update(void)
{
  // TODO: Process the incoming gyro measurements

  /* TODO: Compute the Jacobian of the state propagation matrix f() with
   * respect to the chosen state, given equations (8) and (9) in 
   * Leishman et al, 2014
   */

  /* TODO: Predict the state update using the state propagation function
   * f() using the gyro measurements and estimate of the lumped drag term.
   */

  // TODO: Predict the covariance update using the Jacobian matrix
}

static void ekf_measurement_update(void)
{
  // TODO: Process the incoming accelerometer measurements
 
  // TODO: Compute the Jacobian of the output function h()

  /* TODO: Compute the measurement residual by subtracting the 
   * transformed states, by the output function h(), from the
   * measured accelerometer data.
   */

  /* TODO: Compute the residual covariance using the Jacobian matrix and
   * include the measurement noise covariance R
   */

  // TODO: Compute the Kalman update gain

  /* TODO: Update the state estimate and present it to the user by sending
   * it over an UART port
   */

  /* TODO: Update the covariance estimate
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
}

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro)
{
}