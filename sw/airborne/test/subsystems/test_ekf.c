/*
 * Copyright (C) 2015 Lodewijk Sikkel <l.n.c.sikkel@tudelft.nl>
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
 * accelerometer and gyro measurments.
 */

#include <stdio.h>
#include <stdlib.h>
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
#include "math/pprz_algebra_int.h"

#include "subsystems/abi.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/imu.h"

#include "test/subsystems/algebra.h"

#define DT 1. / PERIODIC_FREQUENCY

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

static void ekf_init(void);
static void ekf_periodic_task(void);
static void ekf_event_task(void);

static abi_event gyro_ev;
static abi_event accel_ev;
struct FloatVect3 accel_m;
static abi_event aligner_ev;
struct FloatRates gyro_bias;
static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel);
static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro);
static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, 
                       struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag);

bool_t IS_ALIGNED = false;

struct StateVector {
  float phi; // rad
  float theta; // rad
  float bp; // rad/s
  float bq; // rad/s
  float br; // rad/s
};

struct StateVector x, x_hat; // state vector, prediction vector

struct FloatRates u, u_unbiased; // input vector (gyro measurements), input vector unbiased

float y[2]; // measurements (accelerometer measurements)

float P[5][5] = {{0}};

float P_hat[5][5] = {{0}};

float F[5][5] = {{0}};

float H[2][5] = {{0}};

float Q[5][5] = {{0}};

float R[2][2] = {{0}};

float I[5][5] = {{0}};

float FP[5][5], PFT[5][5]; // predicted covariance estimate (working matrices)

float HP[2][5], HPHT[2][2]; // innovation covariance (working matrices)

float K[5][2], COF[2][2], PHT[5][2]; // Kalman gain (working matrices)

float KY[5], KH[5][5]; // coverance update (working matrices)

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  // Process the incoming accelerometer measurements
  accel_m.x = ACCEL_FLOAT_OF_BFP(accel->x);
  accel_m.y = ACCEL_FLOAT_OF_BFP(accel->y);

  // RunOnceEvery(10, DOWNLINK_SEND_IMU_ACCEL_SCALED(DefaultChannel, DefaultDevice,
  //                                                 &accel->x,
  //                                                 &accel->y,
  //                                                 &accel->z);
  // );
}

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro)
{
	// Process the incoming gyro measurements
  RATES_FLOAT_OF_BFP(u, *gyro);
  RATES_COPY(u_unbiased, u);

  // RunOnceEvery(10, DOWNLINK_SEND_IMU_GYRO_SCALED(DefaultChannel, DefaultDevice,
  //                                                &gyro->p,
  //                                                &gyro->q,
  //                                                &gyro->r);
  // );
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, 
                       struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  // Use averaged gyro (not truly low-passed) as initial value for bias
  struct FloatRates bias;
  RATES_FLOAT_OF_BFP(bias, *lp_gyro);

  x.bp = bias.p;
  x.bq = bias.q;
  x.br = bias.r;

  IS_ALIGNED = true;
}

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
  AbiBindMsgIMU_GYRO_INT32(0, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(0, &accel_ev, accel_cb);
  AbiBindMsgIMU_LOWPASSED(0, &aligner_ev, aligner_cb);

  mcu_init();

  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);

  imu_init();

  ahrs_aligner_init();

  mcu_int_enable();

  downlink_init();

  ekf_init();
}

static inline void main_periodic_task(void)
{
  RunOnceEvery(100, {
    DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
  });

  if (sys_time.nb_sec > 1) 
    imu_periodic(); 

  if (IS_ALIGNED) 
    ekf_periodic_task();

  // Send EKF debug message
  float dummy_array[1];
  RunOnceEvery(10, {
    DOWNLINK_SEND_TEST_EKF_DEBUG(DefaultChannel, DefaultDevice, &P[0][0], &P[1][1], &P[2][2], &P[3][3], 1, dummy_array);
  }); // called at about 50Hz
  
  RunOnceEvery(10, { LED_PERIODIC();}); // periodic led
}

static inline void main_event_task(void)
{
  mcu_event();

  ImuEvent();
}

static void ekf_time_update(void)
{
  // Unbias the gyro measurements
  u_unbiased.p -= x.bp;
  u_unbiased.q -= x.bq;
  u_unbiased.r -= x.br;

  /* Compute the Jacobian of the state propagation matrix f() with
   * respect to the chosen state, given equations (8) and (9) in 
   * Leishman et al, 2014
   */
  F[0][0] = (cos(x.phi)*u_unbiased.q-sin(x.phi)*u_unbiased.r)*tan(x.theta);
  F[0][1] = 2*(sin(x.phi)*u_unbiased.q+cos(x.phi)*u_unbiased.r)/(cos(2*x.theta)+1);
  F[1][0] = -sin(x.phi)*u_unbiased.q-cos(x.phi)*u_unbiased.r;

  /* Predict the state update using the state propagation function
   * f() using the gyro measurements 
   */
  x_hat.phi = x.phi+(1*u_unbiased.p+(sin(x.phi)*u_unbiased.q+cos(x.phi)*u_unbiased.r)*tan(x.theta))*DT;
  x_hat.theta = x.theta+(cos(x.phi)*u_unbiased.q-sin(x.phi)*u_unbiased.r)*DT;
  x_hat.bp = x.bp;
  x_hat.bq = x.bq;
  x_hat.br = x.br;

  // Predict the covariance update using the Jacobian matrix
  // matmn_mul(5, 5, 5, FP, F, P);
  matmn_mul_transp(5, 5, 5, PFT, P, F);
  // matmn_add(5, 5, FP, PFT);
  // matmn_add(5, 5, FP, Q);
  // matmn_smul(5, 5, FP, DT);
  // matmn_add(5, 5, P_hat, FP);
}

static void ekf_measurement_update(void)
{
  /* Compute the Jacobian of the output function h(), given equations
   * (10) and (11) in Leishman et al, 2014
   */
  H[0][1] = 9.81*cos(x_hat.theta);
  H[1][0] = -9.81*cos(x_hat.phi)*cos(x_hat.theta);
  H[1][1] = 9.81*sin(x_hat.phi)*sin(x_hat.theta);

  /* Compute the measurement residual by subtracting the transformed 
   * states, by the output function h(), from the measured accelerometer data.
   */
  y[0] = accel_m.x-9.81*sin(x_hat.theta);
  y[1] = accel_m.y-(-9.81*sin(x_hat.phi)*cos(x_hat.theta));

  /* Compute the residual covariance using the Jacobian matrix and
   * include the measurement noise covariance R
   */
  matmn_mul(2, 5, 5, HP, H, P_hat);
  matmn_mul_transp(2, 5, 2, HPHT, HP, H);
  matmn_add(2, 2, HPHT, R);

  // Compute the Kalman update gain
  matnn_cof(COF, HPHT, 2);
  matnn_cof_transp(COF, matnn_det(HPHT, 2), 2);
  matmn_mul_transp(5, 5, 2, PHT, P_hat, H);
  matmn_mul(5, 2, 2, K, PHT, COF);

  // Update the state estimate
  matmn_vmul(5, 2, KY, K, y);
  x.phi = x_hat.phi+KY[0];
  x.theta = x_hat.theta+KY[1];

  // Update the covariance estimate
  matmn_mul(5, 2, 5, KH, K, H);
  matmn_smul(5, 5, KH, -1);
  matmn_add(5, 5, KH, I);
  matmn_mul(5, 5, 5, P, KH, P_hat);
}

static void ekf_init(void) {
  matmn_sdiag(5,5,P,1.);

  

  matmn_sdiag(5,5,Q,.0001);

  matmn_sdiag(5,5,R,.0001);

  matmn_sdiag(5,5,I,1.);
}

static void ekf_periodic_task(void)
{
  ekf_time_update();

  // ekf_measurement_update();
}

static void ekf_event_task(void)
{

}