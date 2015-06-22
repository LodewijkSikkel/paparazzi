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

struct StateVector {
  float phi; // rad
  float theta; // rad
  // float u; // m/s
  // float v; // m/s
};

struct StateVector x; // state vector 

float P[2][2] = {
  {1, 0},
  {0, 1},
};

float F[2][2] = {
  {0, 0},
  {0, 0},
};

struct FloatRates u; // input vector (gyro measurements)

float H[2][2] = {
  {0, 0},
  {0, 0},
}; // Jacobian of the output equation

float y[2]; // measurements (accelerometer measurements)

float Q[2][2] = { // process noise
  {.001,    0},
  {   0, .001},
};

float R[2][2] = { // observation noise
  {.001,    0},
  {   0, .001},
};

float I[2][2] = { // process noise
  {1, 0},
  {0, 1},
};

float FP[2][2], PF[2][2], P_hat[2][2]; 

float HP[2][2], HPH[2][2], K[2][2], COF[2][2], HT[2][2], PHT[2][2];

float KY[2], KH[2][2];

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  // TODO: Process the incoming accelerometer measurements
  y[0] = ACCEL_FLOAT_OF_BFP(accel->x);
  y[1] = ACCEL_FLOAT_OF_BFP(accel->y);
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

// Macro to return the transpose of a matrix
static inline void matnn_transp(float (*mat_o)[2], float (*mat_i)[2], uint8_t m, uint8_t n) 
{
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (i != j)
        mat_o[j][i] = mat_i[i][j];
      mat_o[i][j] = mat_i[j][i];
    }
  }
}

// Macro to add two matrices (of similar size) 
static inline void matnn_add(float (*mat_o)[2], float (*mat_i)[2], uint8_t m, uint8_t n)
{
  for (int i = 0; i < m; i++) { // loop over the rows
    for (int j = 0; j < n; j++) { // loop over the columns
      mat_o[i][j] += mat_i[i][j];
    }
  }
}

// Macro to multiply matrix i with matrix ii (similar size)
static inline void matnn_mul(float (*mat_o)[2], 
                      float (*mat_i)[2], 
                      float (*mat_ii)[2], uint8_t m, uint8_t n)
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

// Macro to multiply matrix i with vector v 
static inline void matnn_vmul(float vec_o[2], float (*mat_i)[2], float vec_i[2], uint8_t m, uint8_t n)
{
  for (int i = 0; i < m; i++) { // loop over the rows
    for (int j = 0; j < n; j++) { // loop over the columns of the matrix
      vec_o[i] = mat_i[i][j]*vec_i[j]; 
    }
  }
}

// Macro to multiply matrix i with the transpose of matrix ii
static inline void matnn_mul_transp(float (*mat_o)[2], 
                             float (*mat_i)[2], float (*mat_ii)[2], uint8_t m, uint8_t n) 
{
  for (int i = 0; i < m; i++) { // loop over the rows
    for (int jj = 0; jj < n; jj++) { // loop over the rows of the second matrix
      mat_o[i][jj] = 0;
      for (int j = 0; j < n; j++) { // loop over the columns of the first matrix
        mat_o[i][jj] = mat_i[i][j]*mat_ii[jj][j]; 
      }
    }
  }
}



// Macro to multiply a matrix with a constant
static inline void matnn_smul(float (*mat_o)[2], float scalar, uint8_t m, uint8_t n)
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
static float matnn_det(float mat[][2], uint8_t k) {
  float det = 0;
  float minor[2][2]; // minor matrix

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
static void matnn_cof(float cof[][2], float mat[][2], uint8_t k)
{
  // Allocate an auxilary temporary minor matrix 
  float minor[2][2];  

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
static void matnn_cof_transp(float mat[][2],float det, uint8_t k) 
{
  float elmt;
  for (int i = 1; i < k; i++) {
  	for (int j = 0; j < i; j++) {
      if (i != j) {
  		  elmt = mat[i][j];
  		  mat[i][j] = mat[j][i]/det;
  		  mat[j][i] = elmt/det;
  	  } 
  	}
  }
}

static void ekf_jac_f(void)
{
  /* The elements of the Jacobian matrix that are zero by default are
   * ignored
   */
  F[0][0] = cos(x.phi)*tan(x.theta)*u.q-sin(x.phi)*tan(x.theta)*u.r;
  F[0][1] = 2*(sin(x.phi)*u.q+cos(x.phi)*u.r)/(cos(2*x.theta)+1);
  F[1][0] = -sin(x.phi)*u.q-cos(x.phi)*u.r;
  // F[2][1] = -9.81*cos(x.theta); // gravity hardcoded
  // F[2][3] = u.r;
  // F[3][0] = 9.81*cos(x.phi)*cos(x.theta);
  // F[3][1] = -9.81*sin(x.phi)*sin(x.theta);
  // F[3][2] = -u.r;
}

static void ekf_jac_h(void)
{
  /* The elements of the Jacobian matrix that are zero by default are
   * ignored
   */
  H[0][1] = 9.81*cos(x.theta);
  H[1][0] = -9.81*cos(x.phi)*cos(x.theta);
  H[1][1] = 9.81*sin(x.phi)*sin(x.theta);
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

  // x.u += (-9.81*sin(x.theta)+x.v*u.r)*DT;

  // x.v += (9.81*sin(x.phi)*cos(x.theta)-x.u*u.r)*DT;

  // TODO: Predict the covariance update using the Jacobian matrix
  matnn_mul(FP, F, P, 2, 2);

  matnn_mul_transp(PF, P, F, 2, 2);

  matnn_add(FP, PF, 2, 2);

  matnn_add(FP, Q, 2, 2);

  matnn_smul(FP, DT, 2, 2);

  matnn_add(P_hat, FP, 2, 2);
}

static void ekf_measurement_update(void)
{
  /* TODO: Compute the Jacobian of the output function h(), given equations
   * (10) and (11) in Leishman et al, 2014
   */
  ekf_jac_h();

  /* TODO: Compute the measurement residual by subtracting the 
   * transformed states, by the output function h(), from the
   * measured accelerometer data.
   */
  y[0] -= 9.81*sin(x.theta);
  y[1] -= -9.81*sin(x.phi)*cos(x.theta);

  /* TODO: Compute the residual covariance using the Jacobian matrix and
   * include the measurement noise covariance R
   */
  matnn_mul(HP, H, P_hat, 2, 2);
  matnn_mul_transp(HPH, HP, H, 2, 2); // TODO: Allow this to be done in a single step

  matnn_add(HPH, R, 2, 2);

  // TODO: Compute the Kalman update gain
  matnn_cof(COF, HPH, 2);
  matnn_cof_transp(COF, matnn_det(HPH, 2), 2);
  
  matnn_transp(HT, H, 2, 2);
  matnn_mul_transp(PHT, P_hat, HT, 2, 2);
  matnn_mul(K, PHT, COF, 2, 2);

  // TODO: Update the state estimate
  matnn_vmul(KY, K, y, 2, 2);
  x.phi += KY[0];
  x.theta += KY[1];

  // TODO: Update the covariance estimate
  matnn_mul(KH, K, H, 2, 2);
  matnn_smul(KH, -1, 2, 2);
  matnn_add(KH, I, 2, 2);
  matnn_mul(P, KH, P_hat, 2, 2);
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

  mcu_init();

  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);

  imu_init();

  mcu_int_enable();

  downlink_init();
 
  AbiBindMsgIMU_GYRO_INT32(0, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(0, &accel_ev, accel_cb);

  // TODO: Initialize the state vector
  x.phi = 0;
  x.theta = 0;
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
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      P_dl[k++] = P[i][j];
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
  ekf_measurement_update();

  // TODO: Send the position and velocity estimate of an UART port
}