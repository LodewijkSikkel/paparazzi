/*
 * Copyright (C) 2015 Lodewijk Sikkel <l.n.c.sikkel@tudelft.nl
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
 * @file modules/sensors/dist_imu_mpu6050.c
 * The separate MPU IMUs are are aligned to an arbitrary common body frame
 */

// Include own header
#include "dist_imu_mpu6050.h"

#include <math.h>

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_orientation_conversion.h"
#include "mcu_periph/i2c.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/imu/imu_mpu60x0_defaults.h"

#ifndef IMU_MPU60X0_GYRO_RANGE
#define IMU_MPU60X0_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif

/* MPU60x0 gyro/accel internal lowpass frequency
 * The gyroscope sample rate = output_rate/(1+SMPLRT_DIV), where the output rate 
 * is 8kHz if the cut-off frequency of the low-pass filter is 256Hz. The output
 * rate of the accelerometer is only 1kHz, so oversampling occurs.
 */
#if !defined IMU_MPU60X0_LOWPASS_FILTER && !defined  IMU_MPU60X0_SMPLRT_DIV
#define IMU_MPU60X0_LOWPASS_FILTER MPU60X0_DLPF_256HZ // cut-off frequency of 256Hz
#define IMU_MPU60X0_SMPLRT_DIV 3 // sample-rate divider 
#endif

#ifndef IMU_MPU60X0_GYRO_RANGE
#define IMU_MPU60X0_GYRO_RANGE MPU60X0_GYRO_RANGE_2000
#endif

#ifndef IMU_MPU60X0_ACCEL_RANGE
#define IMU_MPU60X0_ACCEL_RANGE MPU60X0_ACCEL_RANGE_16G
#endif

#ifndef IMU_MPU60X0_I2C_ADDR // if AD0 is pulled low
#define IMU_MPU60X0_I2C_ADDR MPU60X0_ADDR
#endif

#ifndef IMU_MPU60X0_I2C_ADDR_ALT // if AD0 is pulled high
#define IMU_MPU60X0_I2C_ADDR_ALT MPU60X0_ADDR_ALT
#endif

#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN   1
#define IMU_GYRO_R_SIGN   1
#endif

#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN  1
#define IMU_ACCEL_Z_SIGN  1
#endif
 
struct DistImu dist_imu;

float q_sqr; // Just for plotting purposes

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_accel_scaled(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DIST_IMU_ACCEL_SCALED(trans, dev, AC_ID,
                                      &dist_imu.imu[0].accel.x,
                                      &dist_imu.imu[0].accel.y,
                                      &dist_imu.imu[0].accel.z,
                                      &dist_imu.imu[1].accel.x,
                                      &dist_imu.imu[1].accel.y,
                                      &dist_imu.imu[1].accel.z);
}

static void send_accel(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DIST_IMU_ACCEL(trans, dev, AC_ID,
                               &dist_imu.imu[0].accel_f.x, // Change back
                               &dist_imu.imu[0].accel_f.y,
                               &dist_imu.imu[0].accel_f.z);
}

static void send_rates(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DIST_IMU_RATES(trans, dev, AC_ID,
                               &q_sqr,
                               &dist_imu.rates_sqr.q,
                               &dist_imu.rates_sqr.r,
                               &dist_imu.rates_dot.p,
                               &dist_imu.rates_dot.q,
                               &dist_imu.rates_dot.r);
}

#endif

/*
 * This function computes the FIR low-pass filter coefficients using the windowing technique,
 * a Hamming window is used to reduce pass/stop band ripple.
 */
static void fir_lp_comp_coef_init(void) {
  uint32_t i = 0;

  for (int n = -DIST_IMU_FIR_HALF_WINDOW_SIZE; n < DIST_IMU_FIR_HALF_WINDOW_SIZE+1; n++) {
    if (n != 0) { // Skip n = 0 as it will give a division by 0
      dist_imu.fir_lp_coef[i] = 1 / M_PI / n * sin(2. * M_PI * DIST_IMU_FIR_CUTOFF_FREQ / 512. * n); // low-pass filter
      dist_imu.fir_lp_coef[i] *= (.54 - .46 * cos(2 * M_PI * i / ((2 * DIST_IMU_FIR_HALF_WINDOW_SIZE) + 1))); // Hamming window
    }
    i++; 
  }

  dist_imu.fir_lp_coef[DIST_IMU_FIR_HALF_WINDOW_SIZE] = 2. * DIST_IMU_FIR_CUTOFF_FREQ / 512.; // through l'Hopitals rule
}

static void circ_buf_init(struct CircBuf *circ_buf) {
  circ_buf->size = (2*DIST_IMU_FIR_HALF_WINDOW_SIZE)+1;

  for (uint32_t i = 0; i < circ_buf->size; i++)
      FLOAT_VECT3_ZERO(circ_buf->buffer[i]); // clear the buffer 

  circ_buf->head = 0;
  circ_buf->data_available = false;
}

/* 
 * Multiply the circular buffer with a circular coefficient matrix
 */
static void circ_buf_mult_coef(struct FloatVect3 *vo, struct CircBuf *circ_buf, float coef[]) {
  FLOAT_VECT3_ZERO(*vo);

  uint32_t it;
  for (it = 0; it < DIST_IMU_FIR_HALF_WINDOW_SIZE; it++) {
    struct FloatVect3 vt;
    VECT3_SUM(vt, circ_buf->buffer[(circ_buf->head - it) % circ_buf->size], circ_buf->buffer[(circ_buf->head - it + 1) % circ_buf->size]);
    VECT3_ADD_SCALED(*vo, vt, coef[it]);
  }

  VECT3_ADD_SCALED(*vo, circ_buf->buffer[(circ_buf->head - it) % circ_buf->size], coef[DIST_IMU_FIR_HALF_WINDOW_SIZE]);
}

void dist_imu_init(void)
{
  mpu60x0_i2c_init(&dist_imu.mpu[0], &(IMU_MPU60X0_I2C_DEV), IMU_MPU60X0_I2C_ADDR);
  // Change the default configuration
  dist_imu.mpu[0].config.smplrt_div = IMU_MPU60X0_SMPLRT_DIV;
  dist_imu.mpu[0].config.dlpf_cfg = IMU_MPU60X0_LOWPASS_FILTER;
  dist_imu.mpu[0].config.gyro_range = IMU_MPU60X0_GYRO_RANGE;
  dist_imu.mpu[0].config.accel_range = IMU_MPU60X0_ACCEL_RANGE;

  // Compute the DCM
  struct FloatEulers eulers;
  eulers.phi = M_PI;
  eulers.theta = 0;
  eulers.psi = 0;
  orientationSetEulers_f(&dist_imu.imu[0].body_to_imu, &eulers);
  orientationCalcRMat_i(&dist_imu.imu[0].body_to_imu);

  INT_VECT3_ZERO(dist_imu.imu[0].accel_neutral);

  mpu60x0_i2c_init(&dist_imu.mpu[1], &(IMU_MPU60X0_I2C_DEV), IMU_MPU60X0_I2C_ADDR_ALT);
  // Change the default configuration
  dist_imu.mpu[1].config.smplrt_div = IMU_MPU60X0_SMPLRT_DIV;
  dist_imu.mpu[1].config.dlpf_cfg = IMU_MPU60X0_LOWPASS_FILTER;
  dist_imu.mpu[1].config.gyro_range = IMU_MPU60X0_GYRO_RANGE;
  dist_imu.mpu[1].config.accel_range = IMU_MPU60X0_ACCEL_RANGE;

  // Compute the DCM
  eulers.phi = 0;
  eulers.theta = M_PI;
  eulers.psi = 0;
  orientationSetEulers_f(&dist_imu.imu[1].body_to_imu, &eulers);
  orientationCalcRMat_i(&dist_imu.imu[1].body_to_imu);

  INT_VECT3_ZERO(dist_imu.imu[1].accel_neutral);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "DIST_IMU_ACCEL_SCALED", send_accel_scaled);
  register_periodic_telemetry(DefaultPeriodic, "DIST_IMU_ACCEL", send_accel);
  register_periodic_telemetry(DefaultPeriodic, "DIST_IMU_RATES", send_rates);
#endif // DOWNLINK

  fir_lp_comp_coef_init();

  //Initialize the circular FIR buffers
  for (int i = 0; i < DIST_IMU_NB_IMU; i++) 
    circ_buf_init(&dist_imu.circ_buf[i]);
}

void dist_imu_periodic(void)
{
  for (int i = 0; i < DIST_IMU_NB_IMU; i++) {
    mpu60x0_i2c_periodic(&dist_imu.mpu[i]);
  }
}

void dist_imu_event(void)
{
  for (int i = 0; i < DIST_IMU_NB_IMU; i++) {
    mpu60x0_i2c_event(&dist_imu.mpu[i]);
    if (dist_imu.mpu[i].data_available) {
      VECT3_COPY(dist_imu.imu[i].accel_unscaled, dist_imu.mpu[i].data_accel.vect);
      dist_imu.mpu[i].data_available = FALSE;

      // Scale the accelerometer measurements
      imu_scale_accel(&dist_imu.imu[i]);

      // Rotate the local accelerometer frame to the body reference frame
      struct Int32Vect3 accel;
      int32_rmat_transp_vmult(&accel, &dist_imu.imu[i].body_to_imu.rmat_i, &dist_imu.imu[i].accel);
      ACCELS_FLOAT_OF_BFP(dist_imu.imu[i].accel_f, accel);

      // // Apply a FIR low pass filter to the measured data
      // if (!dist_imu.circ_buf[i].data_available) {
      //   dist_imu.circ_buf[i].buffer[0] = dist_imu.imu[i].accel_f; // add the measurement to the first position of the buffer
      //   dist_imu.circ_buf[i].data_available = true;
      // } else {
      //   dist_imu.circ_buf[i].head = (dist_imu.circ_buf[i].head + 1) % dist_imu.circ_buf[i].size;          
      //   dist_imu.circ_buf[i].buffer[dist_imu.circ_buf[i].head] = dist_imu.imu[i].accel_f; // add the measurement to the head position of the buffer
      //   circ_buf_mult_coef(&dist_imu.imu[i].accel_f, &dist_imu.circ_buf[i], dist_imu.fir_lp_coef);
      // }
    }
  }

  /* 
   * Compute the angular rates squared and angular acceleration, the
   * inverse of the kinematic matrix is done in MATLAB
   */
  dist_imu.accel.x = .6*dist_imu.imu[0].accel_f.x+.1333*dist_imu.imu[0].accel_f.z+
    .4*dist_imu.imu[1].accel_f.x-.1333*dist_imu.imu[1].accel_f.z;
  dist_imu.accel.z = -.1333*dist_imu.imu[0].accel_f.x+.6*dist_imu.imu[0].accel_f.z+
    .1333*dist_imu.imu[1].accel_f.x+.4*dist_imu.imu[1].accel_f.z;
  dist_imu.rates_sqr.q = -13.3333*dist_imu.imu[0].accel_f.x+13.3333*dist_imu.imu[1].accel_f.x;
  dist_imu.rates_dot.q = -13.3333*dist_imu.imu[0].accel_f.z+13.3333*dist_imu.imu[1].accel_f.z; 

  q_sqr = RATE_FLOAT_OF_BFP(imu.gyro.q)*RATE_FLOAT_OF_BFP(imu.gyro.q);
}
