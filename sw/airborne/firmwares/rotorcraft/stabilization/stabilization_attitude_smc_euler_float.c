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
 * @file stabilization_attitude_euler_float.c
 *
 * TO DO
 */

#include <stdio.h>

#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

#include "math/pprz_algebra_float.h"
#include "std.h"
#include "paparazzi.h"
#include "state.h"
#include "generated/airframe.h"
#include "subsystems/abi.h"
#include "subsystems/imu.h"
#include "subsystems/actuators/motor_mixing.h"

#define STABILIZATION_ATTITUDE_SERVO_MIN 3000

#define STABILIZATION_ATTITUDE_SERVO_MAX 12000

#define STABILIZATION_ATTITUDE_FILT_OMEGA_SQR ( STABILIZATION_ATTITUDE_FILT_OMEGA * STABILIZATION_ATTITUDE_FILT_OMEGA )

 #define STABILIZATION_ATTITUDE_FILT_OMEGA_SQR_R ( STABILIZATION_ATTITUDE_FILT_OMEGA_R * STABILIZATION_ATTITUDE_FILT_OMEGA_R )

#define STABILIZATION_ATTITUDE_FILT_DT ( 1 / 512. )

struct FloatEulers *att_float; 
struct FloatEulers att_err;

struct FloatRates *rate_float;
struct FloatRates rate_err;

abi_event rpm_ev;
float rpm_scaled[ACTUATORS_NB];
static void rpm_cb(uint8_t sender_id, uint16_t *rpm, uint8_t count);

#define MOTOR_SCALING_VALUE ( 1. / MOTOR_MIXING_NB_MOTOR ) 

#ifndef STABILIZATION_ATTITUDE_DEADBAND_TAU
#define STABILIZATION_ATTITUDE_DEADBAND_TAU 0.
#endif

float stabilization_attitude_deadband_tau = STABILIZATION_ATTITUDE_DEADBAND_TAU;

#ifndef STABILIZATION_ATTITUDE_LAMBDA_0 
#error LAMBDA_0 has to be defined 
#endif

struct FloatRates stabilization_attitude_lambda_0 = STABILIZATION_ATTITUDE_LAMBDA_0;

struct FloatRates stabilization_attitude_lambda_1 = { 1., 1., 1. };

struct FloatRates filt_ang_rate = {0., 0., 0.};
struct FloatRates filt_ang_rate_dot = {0., 0., 0.};
struct FloatRates filt_ang_rate_ddot = {0., 0., 0.};

struct FloatRates u_act;
struct FloatRates filt_u_act = {0., 0., 0.};
struct FloatRates filt_u_act_dot = {0., 0., 0.};
struct FloatRates filt_u_act_ddot = {0., 0., 0.};

struct FloatRates h;
struct FloatRates slid_func;
struct FloatRates slid_contr_incr;
struct FloatRates u = {0., 0., 0.};

#ifndef STABILIZATION_ATTITUDE_INV_INPUT_DISTR_P
#error STABILIZATION_ATTITUDE_INV_INPUT_DISTR_P has to be defined 
#endif 

#ifndef STABILIZATION_ATTITUDE_INV_INPUT_DISTR_Q
#error STABILIZATION_ATTITUDE_INV_INPUT_DISTR_Q has to be defined 
#endif 

#ifndef STABILIZATION_ATTITUDE_INV_INPUT_DISTR_R
#error STABILIZATION_ATTITUDE_INV_INPUT_DISTR_R has to be defined 
#endif 

struct FloatRates stabilization_attitude_inv_input_distr = 
    { STABILIZATION_ATTITUDE_INV_INPUT_DISTR_P, STABILIZATION_ATTITUDE_INV_INPUT_DISTR_Q, 0.};

/* The initial gain values */
#ifndef STABILIZATION_ATTITUDE_K
#error STABILIZATION_ATTITUDE_K has to be defined 
#endif 

struct FloatRates stabilization_attitude_k = STABILIZATION_ATTITUDE_K;

/* Adaption parameters */
#ifndef STABILIZATION_ATTITUDE_ETA
#error STABILIZATION_ATTITUDE_ETA has to be defined
#endif

struct FloatRates stabilization_attitude_eta = STABILIZATION_ATTITUDE_ETA;

#ifndef STABILIZATION_ATTITUDE_K_MIN
#error STABILIZATION_ATTITUDE_K_MIN has to be defined 
#endif 

struct FloatRates stabilization_attitude_k_min = STABILIZATION_ATTITUDE_K_MIN;

struct FloatRates abs_slid_func_prev = {0., 0., 0.};

uint8_t stabilization_attitude_counter[3] = {0, 0, 0};

#ifndef STABILIZATION_ATTITUDE_N
#error STABILIZATION_ATTITUDE_N has to be defined
#endif

uint8_t stabilization_attitude_N = STABILIZATION_ATTITUDE_N;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_att_smc(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE_SMC(trans, dev, AC_ID,
                                  &(att_float->phi), &(att_float->theta),
                                  &stab_att_sp_euler.phi, &stab_att_sp_euler.theta,
                                  &(rate_float->p), &(rate_float->q),
                                  &slid_func.p, &slid_func.q,
                                  &stabilization_attitude_k.p, &stabilization_attitude_k.q,
                                  &u.p, &u.q);
}
#endif

void stabilization_attitude_init(void)
{
  stabilization_attitude_ref_init();

  // Register to RPM messages
  AbiBindMsgRPM(ABI_BROADCAST, &rpm_ev, rpm_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_SMC", send_att_smc);
#endif
}

void stabilization_attitude_read_rc(bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn)
{
  stabilization_attitude_read_rc_setpoint_eulers_f(&stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
}

void stabilization_attitude_enter(void)
{
  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_f();

  stabilization_attitude_ref_enter();
}

void stabilization_attitude_set_failsafe_setpoint(void)
{
  stab_att_sp_euler.phi = 0.0;
  stab_att_sp_euler.theta = 0.0;
  stab_att_sp_euler.psi = stateGetNedToBodyEulers_f()->psi;
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  EULERS_FLOAT_OF_BFP(stab_att_sp_euler, *rpy);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  struct FloatVect2 cmd_f;
  cmd_f.x = ANGLE_FLOAT_OF_BFP(cmd->x);
  cmd_f.y = ANGLE_FLOAT_OF_BFP(cmd->y);

  /* Rotate horizontal commands to body frame by psi */
  float psi = stateGetNedToBodyEulers_f()->psi;
  float s_psi = sinf(psi);
  float c_psi = cosf(psi);
  stab_att_sp_euler.phi = -s_psi * cmd_f.x + c_psi * cmd_f.y;
  stab_att_sp_euler.theta = -c_psi * cmd_f.x - s_psi * cmd_f.y;
  stab_att_sp_euler.psi = ANGLE_FLOAT_OF_BFP(heading);
}

static void stabilization_rate_filter(void) {
  filt_ang_rate.p = filt_ang_rate.p + filt_ang_rate_dot.p * STABILIZATION_ATTITUDE_FILT_DT;
  filt_ang_rate.q = filt_ang_rate.q + filt_ang_rate_dot.q * STABILIZATION_ATTITUDE_FILT_DT;
  filt_ang_rate.r = filt_ang_rate.r + filt_ang_rate_dot.r * STABILIZATION_ATTITUDE_FILT_DT;
  
  filt_ang_rate_dot.p = filt_ang_rate_dot.p + filt_ang_rate_ddot.p * STABILIZATION_ATTITUDE_FILT_DT;
  filt_ang_rate_dot.q = filt_ang_rate_dot.q + filt_ang_rate_ddot.q * STABILIZATION_ATTITUDE_FILT_DT;
  filt_ang_rate_dot.r = filt_ang_rate_dot.r + filt_ang_rate_ddot.r * STABILIZATION_ATTITUDE_FILT_DT;
  
  filt_ang_rate_ddot.p = -filt_ang_rate_dot.p * 2 * STABILIZATION_ATTITUDE_FILT_ZETA * STABILIZATION_ATTITUDE_FILT_OMEGA 
                      + ( stateGetBodyRates_f()->p - filt_ang_rate.p ) * STABILIZATION_ATTITUDE_FILT_OMEGA_SQR;
  filt_ang_rate_ddot.q = -filt_ang_rate_dot.q * 2 * STABILIZATION_ATTITUDE_FILT_ZETA * STABILIZATION_ATTITUDE_FILT_OMEGA 
                      + ( stateGetBodyRates_f()->q - filt_ang_rate.q ) * STABILIZATION_ATTITUDE_FILT_OMEGA_SQR;
  filt_ang_rate_ddot.r = -filt_ang_rate_dot.r * 2 * STABILIZATION_ATTITUDE_FILT_ZETA_R * STABILIZATION_ATTITUDE_FILT_OMEGA_R 
                      + ( stateGetBodyRates_f()->r - filt_ang_rate.r ) * STABILIZATION_ATTITUDE_FILT_OMEGA_SQR_R;
}

static void stabilization_input_filter(void)
{
  u_act.p = ( -rpm_scaled[0] + rpm_scaled[1] + rpm_scaled[2] - rpm_scaled[3] ) * MOTOR_SCALING_VALUE;
  u_act.q = (  rpm_scaled[0] + rpm_scaled[1] - rpm_scaled[2] - rpm_scaled[3] ) * MOTOR_SCALING_VALUE;
  u_act.r = (  rpm_scaled[0] - rpm_scaled[1] + rpm_scaled[2] - rpm_scaled[3] ) * MOTOR_SCALING_VALUE;

  filt_u_act.p = filt_u_act.p + filt_u_act_dot.p * STABILIZATION_ATTITUDE_FILT_DT;
  filt_u_act.q = filt_u_act.q + filt_u_act_dot.q * STABILIZATION_ATTITUDE_FILT_DT;
  filt_u_act.r = filt_u_act.r + filt_u_act_dot.r * STABILIZATION_ATTITUDE_FILT_DT;

  filt_u_act_dot.p = filt_u_act_dot.p + filt_u_act_ddot.p * STABILIZATION_ATTITUDE_FILT_DT;
  filt_u_act_dot.q = filt_u_act_dot.q + filt_u_act_ddot.q * STABILIZATION_ATTITUDE_FILT_DT;
  filt_u_act_dot.r = filt_u_act_dot.r + filt_u_act_ddot.r * STABILIZATION_ATTITUDE_FILT_DT;

  filt_u_act_ddot.p = -filt_u_act_dot.p * 2  * STABILIZATION_ATTITUDE_FILT_ZETA * STABILIZATION_ATTITUDE_FILT_OMEGA 
                      + (u_act.p - filt_u_act.p) * STABILIZATION_ATTITUDE_FILT_OMEGA_SQR;
  filt_u_act_ddot.q = -filt_u_act_dot.q * 2  * STABILIZATION_ATTITUDE_FILT_ZETA * STABILIZATION_ATTITUDE_FILT_OMEGA 
                      + (u_act.q - filt_u_act.q) * STABILIZATION_ATTITUDE_FILT_OMEGA_SQR;
  filt_u_act_ddot.r = -filt_u_act_dot.r * 2  * STABILIZATION_ATTITUDE_FILT_ZETA_R * STABILIZATION_ATTITUDE_FILT_OMEGA_R
                      + (u_act.r - filt_u_act.r) * STABILIZATION_ATTITUDE_FILT_OMEGA_SQR_R;
}

#define BOUND_CONTROLS(_v, _min, _max)           \
{                                                \
  _v = _v < _min ? _min : _v > _max ? _max : _v; \
} 

static void rpm_cb(uint8_t sender_id, uint16_t *rpm, uint8_t count)
{
  /** index 0: top right motor
   *  index 1: top left motor
   *  index 2: bottom left motor
   *  index 3: bottom right motor 
   */
  for (uint8_t i = 0; i < count; i++) {
    rpm_scaled[i] = (rpm[i] - get_servo_min(i));
    rpm_scaled[i] *= (MAX_PPRZ / (float)(get_servo_max(i)-get_servo_min(i)));
  }
}

static int8_t stabilization_smc_get_adaptation_value(float abs_value, float *abs_value_prev, uint8_t* counter)
{
  // The previous sample as well as the current sample lie within the boundary
  if ( abs_value <= stabilization_attitude_deadband_tau && 
        *abs_value_prev <= stabilization_attitude_deadband_tau ) {
    if ( ++(*counter) > stabilization_attitude_N ) {
       *counter = 0;
       *abs_value_prev = abs_value;
       return 1;
    }
  }
  // The previous sample as well as the current sample lie within the boundary
  else if (abs_value > stabilization_attitude_deadband_tau &&
        *abs_value_prev > stabilization_attitude_deadband_tau ) { 
    if ( ++(*counter) > stabilization_attitude_N ) {
      *counter = 0;
      *abs_value_prev = abs_value;
      return -1;
    }
  } 
  // The previous sample does not lie within the same region as the current sample
  else {
    *counter = 0;
  }

  // Set the previous sample to be equal to the current sample
  *abs_value_prev = abs_value;

  return 0;
} 

static void stabilization_attitude_cmd(void) {
  // Update the input filter
  stabilization_input_filter();

  // Compute the rate feedback
  h.p = stabilization_attitude_lambda_0.p * stateGetBodyRates_f()->p
        + stabilization_attitude_lambda_1.p * filt_ang_rate_dot.p;
  h.q = stabilization_attitude_lambda_0.q * stateGetBodyRates_f()->q
        + stabilization_attitude_lambda_1.q * filt_ang_rate_dot.q;
  // h.r = stabilization_attitude_lambda_0.r * filt_ang_rate.r
  //       + stabilization_attitude_lambda_1.r * filt_ang_rate_dot.r;

  slid_func.p = stabilization_attitude_lambda_0.p * att_err.phi
                + stabilization_attitude_lambda_1.p * rate_float->p;
  slid_func.q = stabilization_attitude_lambda_0.q * att_err.theta
                + stabilization_attitude_lambda_1.q * rate_float->q;
  // slid_func.r = stabilization_attitude_lambda_0.r * -1 * att_err.psi
  //               + stabilization_attitude_lambda_1.r * -1 * rate_err.r;

  /**
   * Gain adaption
   */
  stabilization_attitude_k.p = stabilization_attitude_k.p + ((stabilization_attitude_k.p <= stabilization_attitude_k_min.p) ? 
                                stabilization_attitude_eta.p : -1. * stabilization_attitude_eta.p * stabilization_smc_get_adaptation_value(fabs(slid_func.p), &abs_slid_func_prev.p, &stabilization_attitude_counter[0]))*
                                STABILIZATION_ATTITUDE_FILT_DT;

  stabilization_attitude_k.q = stabilization_attitude_k.q + ((stabilization_attitude_k.q <= stabilization_attitude_k_min.q) ? 
                                stabilization_attitude_eta.q : -1. * stabilization_attitude_eta.q * stabilization_smc_get_adaptation_value(fabs(slid_func.q), &abs_slid_func_prev.q, &stabilization_attitude_counter[1]))*
                                STABILIZATION_ATTITUDE_FILT_DT;       

  // stabilization_attitude_k.r = stabilization_attitude_k.r * stabilization_attitude_eta.r * 
  //                              -stabilization_smc_get_adaptation_value(f_abs(slid_func.r), &stabilization_attitude_counter[2]);   

  slid_contr_incr.p = -( h.p + stabilization_attitude_k.p * tanh( slid_func.p ) ) * 
                        stabilization_attitude_inv_input_distr.p / stabilization_attitude_lambda_1.p;
  slid_contr_incr.q = -( h.q + stabilization_attitude_k.q * tanh( slid_func.q ) ) * 
                        stabilization_attitude_inv_input_distr.q / stabilization_attitude_lambda_1.q;
  // slid_contr_incr.r = -(h.r + stabilization_attitude_k.q * stabilization_smc_sign(slid_func.r))
  //                     * STABILIZATION_ATTITUDE_INV_INPUT_DISTR_R;

  // u.p = filt_u_act.p + slid_contr_incr.p;
  // u.q = filt_u_act.q + slid_contr_incr.q;
  // u.r = filt_u_act.r + slid_contr_incr.r;

  u.r = STABILIZATION_ATTITUDE_PSI_PGAIN * -att_err.psi + STABILIZATION_ATTITUDE_PSI_DGAIN * rate_err.r;

  BOUND_CONTROLS(u.p, -4500., 4500.);
  BOUND_CONTROLS(u.q, -4500., 4500.);
  float half_thrust = ((float) stabilization_cmd[COMMAND_THRUST]/2);
  BOUND_CONTROLS(u.r, -half_thrust, half_thrust);

  // Do not increment if thrust is off
  if(stabilization_cmd[COMMAND_THRUST] <= 300) {
    FLOAT_RATES_ZERO(slid_contr_incr);
    FLOAT_RATES_ZERO(u);
  }

  /* Set the output commands */
  stabilization_cmd[COMMAND_ROLL] = u.p;
  stabilization_cmd[COMMAND_PITCH] = u.q;
  stabilization_cmd[COMMAND_YAW] = u.r;
}

void stabilization_attitude_run(bool_t  in_flight)
{
  /* Update the reference trajectory generator */
  // stabilization_attitude_ref_update();

  /* Propagate the second order filter on the gyroscopes */
  stabilization_rate_filter();

  /* Compute feedback */
  /* attitude error */
  att_float = stateGetNedToBodyEulers_f();
  EULERS_DIFF(att_err, *att_float, stab_att_sp_euler); /* c = a - b */
  FLOAT_ANGLE_NORMALIZE(att_err.psi);

  /* rate error */
  rate_float = stateGetBodyRates_f();
  RATES_DIFF(rate_err, stab_att_ref_rate, *rate_float)

  stabilization_attitude_cmd();

  // /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
}
