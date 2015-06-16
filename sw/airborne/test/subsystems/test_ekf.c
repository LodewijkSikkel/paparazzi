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
 *
 * Assume the followign system state vector: x = [phi theta u v]^T and 
 * input vector: u = [p q r]^T
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
#include "subsystems/datalink/downlink.h"

#include "subsystems/imu.h"
#include "subsystems/abi.h"

#define DT 1/512.

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

static void ekf_jac_f(void);
static void ekf_jac_h(void);
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

static inline void matnn_add(float (*mat_o)[4], float (*mat_i)[4], uint8_t m, uint8_t n);
static inline void matnn_mul(float (*mat_o)[4], float (*mat_i)[4], float (*mat_ii)[4], uint8_t m, uint8_t n);
static void matnn_mul_transp(float (*mat_o)[4], float (*mat_i)[4], float (*mat_ii)[4], uint8_t m, uint8_t n);
static void matnn_smul(float (*mat_o)[4], float scalar, uint8_t m, uint8_t n);
static float matnn_det(float (*mat)[4], uint8_t k);
static void matnn_cof(float (*mat)[4], float (*cof)[4], uint8_t k);
static void matnn_transp(float (*mat)[4], uint8_t k);

struct StateVector {
  float phi; // rad
  float theta; // rad
  float u; // m/s
  float v; // m/s
};

struct StateVector x; // state vector 

float P[4][4] = {
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1},
};

float F[4][4] = {
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
}; // Jacobian of the state propagation matrix, initialized to 0

struct FloatRates u; // input vector (gyro measurements)

float H[4][3] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
}; // Jacobian of the output equation

struct FloatVect3 y; // measurements (accelerometer measurements)

float Q[4][4] = { // process noise
  {.001,    0,    0,    0},
  {   0, .001,    0,    0},
  {   0,    0, .001,    0},
  {   0,    0,    0, .001},
};

float R[4][4] = { // observation noise
  {.001,    0,    0,    0},
  {   0, .001,    0,    0},
  {   0,    0, .001,    0},
  {   0,    0,    0, .001},
};

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

  x.phi = 0;
  x.theta = 0;
  x.u = 0;
  x.v = 0;
}

static inline void main_periodic_task(void)
{
  RunOnceEvery(100, {
    DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
  });

  if (sys_time.nb_sec > 1) { 
  	imu_periodic(); 
  }

  float P_dl[16];
  int k = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      P_dl[k++] = F[i][j];
    }
  }
  RunOnceEvery(50, {
  	DOWNLINK_SEND_TEST_EKF_DEBUG(DefaultChannel, DefaultDevice, &x.phi, &x.theta, 16, P_dl);
  }); // called at about 10Hz
  
  RunOnceEvery(10, { LED_PERIODIC();}); // periodic led
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
  // ekf_measurement_update();

  // TODO: Send the position and velocity estimate of an UART port
}

static void ekf_jac_f(void)
{
  /* The elements of the Jacobian matrix that are zero by default are
   * ignored
   */
  F[0][0] = cos(x.phi)*tan(x.theta)*u.q-sin(x.phi)*tan(x.theta)*u.r;
  F[0][1] = 2*(sin(x.phi)*u.q+cos(x.phi)*u.r)/(cos(2*x.theta)+1);
  F[1][0] = -sin(x.phi)*u.q-cos(x.phi)*u.r;
  F[2][1] = -9.81*cos(x.theta); // gravity hardcoded
  F[2][3] = u.r;
  F[3][0] = 9.81*cos(x.phi)*cos(x.theta);
  F[3][1] = -9.81*sin(x.phi)*sin(x.theta);
  F[3][2] = -u.r;
}

static void ekf_jac_h(void)
{
  /* The elements of the Jacobian matrix that are zero by default are
   * ignored
   */
  H[0][0] = 1;
  H[0][1] = sin(x.phi)*tan(x.theta);
  H[0][2] = cos(x.phi)*tan(x.theta);
  H[1][1] = cos(x.phi);
  H[1][2] = -sin(x.phi);
  H[2][2] = x.v;
  H[3][2] = -x.u;
}

static void ekf_time_update(void)
{
  /* TODO: Compute the Jacobian of the state propagation matrix f() with
   * respect to the chosen state, given equations (8) and (9) in 
   * Leishman et al, 2014
   */
  ekf_jac_f();

  /* TODO: Predict the state update using the state propagation function
   * f() using the gyro measurements and estimate of the lumped drag term.
   */
  x.phi += (1*u.p+sin(x.phi)*tan(x.theta)*u.q+cos(x.phi)*tan(x.theta)*u.r)*DT;

  x.theta += (cos(x.phi)*u.q-sin(x.phi)*u.r)*DT;

  x.u += (-9.81*sin(x.theta)+x.v*u.r)*DT;

  x.v += (9.81*sin(x.phi)*cos(x.theta)-x.u*u.r)*DT;

  // TODO: Predict the covariance update using the Jacobian matrix
  float FP[4][4], PF[4][4], Ft[4][4]; 
  
  matnn_mul(FP, F, P, 4, 4);

  // TODO (urgent): change this to the matnn_mult_transp
  memcpy(Ft, F, 16*sizeof(float));
  // matnn_transp(Ft, 4);
  // matnn_mul(PF, P, Ft, 4, 4);

  matnn_add(P, Ft, 4, 4);

  // // TODO (urgent): allow for multiple matrices to be added together
  // matnn_add(FP, PF, 4, 4);

  // matnn_add(FP, Q, 4, 4);

  // matnn_smul(FP, DT, 4, 4);

  // matnn_add(P, FP, 4, 4);
}

static void ekf_measurement_update(void)
{
  /* TODO: Compute the Jacobian of the output function h(), given equations
   * (10) and (11) in Leishman et al, 2014

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

  // TODO: Update the covariance estimate
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  // TODO: Process the incoming accelerometer measurements
  ACCELS_FLOAT_OF_BFP(y, *accel);
}

static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro)
{
	// TODO: Process the incoming gyro measurements
  u.p = RATE_FLOAT_OF_BFP(gyro->p);
  u.q = RATE_FLOAT_OF_BFP(gyro->q);
  u.r = RATE_FLOAT_OF_BFP(gyro->r);
}

// Macro to add two matrices (of similar size) 
static void matnn_add(float (*mat_o)[4], float (*mat_i)[4], uint8_t m, uint8_t n)
{
  for (int i = 0; i < m; i++) { // loop over the rows
    for (int j = 0; j < n; j++) { // loop over the columns
      mat_o[i][j] += mat_i[i][j];
    }
  }
}

// Macro to multiply two matrices
static void matnn_mul(float (*mat_o)[4], float (*mat_i)[4], float (*mat_ii)[4], uint8_t m, uint8_t n)
{
  for (int i = 0; i < m; i++) { // loop over the rows
    for (int jj = 0; jj < n; jj++) { // loop over the columns of the second matrix
      mat_o[i][jj] = 0;
      for (int j = 0; j < n; j++) { // loop over the columns of the first matrix
        mat_o[i][jj] = mat_i[i][j]*mat_ii[j][jj]; 
      }
    }
  }
}

// Macro to multiply a matrix with the transpose of the other matrix
static void matnn_mul_transp(float (*mat_o)[4], float (*mat_i)[4], float (*mat_ii)[4], uint8_t m, uint8_t n) 
{
}


// Macro to multiply a matrix with a constant
static void matnn_smul(float (*mat_o)[4], float scalar, uint8_t m, uint8_t n)
{
  for (int i = 0; i < m; i++) { // loop over the rows
    for (int j = 0; j < n; j++) { // loop over the columns
      mat_o[i][j] = mat_o[i][j]*scalar; 
    }
  }
}

/*
 * Macro to compute the determinant of a n x n matrix, due to the
 * limitation of libc on the stm32 the dynamic matrix allocation has
 * been removed.
 */
static float matnn_det(float mat[][4], uint8_t k) {
  float det = 0;
  float minor[4][4]; // minor matrix

  int i, j, c, m, n;

  if (k < 2) { // this should never be called
  	return 0;
  } else if (k == 2) {
	  return det = mat[0][0]*mat[1][1]-mat[1][0]*mat[0][1];
  } else {
   	for (c = 0; c < k; c++) { // define the working column
      m = 0;
      n = 0;
  	  for (i = 0; i < k; i++) { // loop over the rows
    		for (j = 0; j < k; j++) {
          minor[i][j] = 0;
    		  if (i != 0 && j != c) { // check if the current column unequal to the working column
      		  minor[m][n] = mat[i][j]; // fill the minor matrix from the first element
            if (n < (k-2)) // define the index of the minor matrix
              n++;
            else {
              n = 0;
              m++;
            }
          }
    		}
  	  }
  	  det += pow(-1,c)*mat[0][c]*matnn_det(minor, k-1); // compute the determinant
	  }
  }
  return det;
}

// Macro that will fill in the cofactor matrix
static void matnn_cof(float mat[][4], float cof[][4], uint8_t k)
{
  // Allocate an auxilary temporary minor matrix 
  float minor[4][4];  

  int i, ii, j, jj, m, n;

  for (i = 0; i < k; i++) { // define the working row
  	for (j = 0; j < k; j++) { // define the working column
      m = 0;
      n = 0;
      for (ii = 0; ii < k; ii++) { // loop over the rows of mat
        for (jj = 0; jj < k; jj++) { // loop over the columns of mat
          if (i != ii && j != jj) { // skip the working column abd working row
            minor[m][n] = mat[ii][jj];
            if (n < (k-2)) // Define the index of the minor matrix
                n++;
            else {
              n = 0;
              m++;
            }
          }
        }
      }
      cof[i][j] = pow(-1.,(i+j))*matnn_det(minor, k-1); // compute the cofactor element
  	}
  }
}

// 
static void matnn_transp(float mat[][4], uint8_t k) 
{
  float elmt;
  for (int i = 1; i < k; i++) {
  	for (int j = 0; j < i; j++) {
      if (i != j) {
  		  elmt = mat[i][j];
  		  mat[i][j] = mat[j][i];
  		  mat[j][i] = elmt;
  	  } 
  	}
  }
}

