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

/** @file stabilization_attitude_quat_indi.c
 *  NEEDS TO BE UPDATED: UGLY CODE
 */

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"
#include "subsystems/actuators/motor_mixing.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

int32_t stabilization_att_smc_cmd[COMMANDS_NB];

#define STABILIZATION_ATTITUDE_FILT_OMEGA_SQR ( STABILIZATION_ATTITUDE_FILT_OMEGA * STABILIZATION_ATTITUDE_FILT_OMEGA )

#define STABILIZATION_ATTITUDE_FILT_OMEGA_SQR_R ( STABILIZATION_ATTITUDE_FILT_OMEGA_R * STABILIZATION_ATTITUDE_FILT_OMEGA_R )

#define STABILIZATION_ATTITUDE_FILT_DT ( 1 / 512. )

struct FloatRates filt_ang_rate = { 0., 0., 0. };
struct FloatRates filt_ang_rate_dot = { 0., 0., 0. };
struct FloatRates filt_ang_rate_ddot = { 0., 0., 0. };

struct Int32Quat* att_quat = 0; 

struct Int32Quat att_err = {0, 0, 0, 0};

struct Int32Rates rate_err = {0, 0, 0};

struct FloatRates s_func = {0., 0., 0.};

#define NUM_MOTORS 4

#define MOTOR_SCALING_VALUE ( 1. / 4. ) 

uint16_t rpm_motor[NUM_MOTORS] = { 0, 0, 0, 0 };

struct FloatRates u_act = { 0., 0., 0. };
struct FloatRates filt_u_act = { 0., 0., 0. };
struct FloatRates filt_u_act_dot = { 0., 0., 0. };
struct FloatRates filt_u_act_ddot = { 0., 0., 0. };

struct FloatRates u = { 0., 0., 0. };

struct FloatRates s_contr_incr = { 0., 0., 0. };

#ifndef STABILIZATION_ATTITUDE_DEADBAND_TAU
#define STABILIZATION_ATTITUDE_DEADBAND_TAU 0.
#endif

float stabilization_attitude_deadband_tau = STABILIZATION_ATTITUDE_DEADBAND_TAU;

#ifndef STABILIZATION_ATTITUDE_LAMBDA_0 
#error LAMBDA_0 has to be defined 
#endif

struct FloatVect3 stabilization_attitude_lambda_0 = STABILIZATION_ATTITUDE_LAMBDA_0;

struct Int8Vect3 stabilization_attitude_lambda_1 = { 1., 1., 1. };

struct FloatRates h = { 0., 0., 0. };

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

#ifndef STABILIZATION_ATTITUDE_K
#error STABILIZATION_ATTITUDE_K has to be defined 
#endif 

struct FloatVect3 stabilization_attitude_k = STABILIZATION_ATTITUDE_K;

#define GAIN_PRESCALER_P 48
#define GAIN_PRESCALER_D 48

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ahrs_ref_quat(struct transport_tx *trans, struct link_device *dev) 
{
  struct Int32Quat* quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_REF_QUAT(trans, dev, AC_ID,
      &stab_att_ref_quat.qi,
      &stab_att_ref_quat.qx,
      &stab_att_ref_quat.qy,
      &stab_att_ref_quat.qz,
      &(quat->qi),
      &(quat->qx),
      &(quat->qy),
      &(quat->qz));
}

static void send_att_sliding_mode_control(struct transport_tx *trans, struct link_device *dev) 
{
  pprz_msg_send_SMC(trans, dev, AC_ID,
      &h.p,
      &h.q,
      &h.r,
      &att_err.qx,
      &att_err.qy,
      &att_err.qz,
      &rate_err.p,
      &rate_err.q,
      &rate_err.r,
      &s_func.p,
      &s_func.q,
      &s_func.r,
      &s_contr_incr.p,
      &s_contr_incr.q,
      &s_contr_incr.r,
      &u.p,
      &u.q,
      &u.r,
      &att_quat->qx,
      &att_quat->qy,
      &att_quat->qz,
      &stab_att_sp_quat.qx,
      &stab_att_sp_quat.qy,
      &stab_att_sp_quat.qz);
}

#endif

void stabilization_attitude_init(void) {

  stabilization_attitude_ref_init();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "AHRS_REF_QUAT", send_ahrs_ref_quat);
  register_periodic_telemetry(DefaultPeriodic, "SMC", send_att_sliding_mode_control);
#endif
}

void stabilization_attitude_enter(void) {

  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  stabilization_attitude_ref_enter();

  FLOAT_RATES_ZERO(filt_ang_rate);
  FLOAT_RATES_ZERO(filt_ang_rate_dot);
  FLOAT_RATES_ZERO(filt_ang_rate_ddot);

  FLOAT_RATES_ZERO(u);
}

void stabilization_attitude_set_failsafe_setpoint(void) {
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy) {
  // stab_att_sp_euler.psi still used in ref..
  memcpy(&stab_att_sp_euler, rpy, sizeof(struct Int32Eulers));

  quat_from_rpy_cmd_i(&stab_att_sp_quat, &stab_att_sp_euler);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading) {
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler.psi = heading;

  // compute sp_euler phi/theta for debugging/telemetry
  /* Rotate horizontal commands to body frame by psi */
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  stab_att_sp_euler.phi = (-s_psi * cmd->x + c_psi * cmd->y) >> INT32_TRIG_FRAC;
  stab_att_sp_euler.theta = -(c_psi * cmd->x + s_psi * cmd->y) >> INT32_TRIG_FRAC;

  quat_from_earth_cmd_i(&stab_att_sp_quat, cmd, heading);
}

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

#define BOUND_CONTROLS(_v, _min, _max)           \
{                                                \
  _v = _v < _min ? _min : _v > _max ? _max : _v; \
}                                                \

static float stabilization_smc_sign(float value) 
{
  return value < -stabilization_attitude_deadband_tau ? -1 : ( value > +stabilization_attitude_deadband_tau ? 1 : 0 );
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
  // filt_ang_rate_ddot.r = -filt_ang_rate_dot.r * 2 * STABILIZATION_ATTITUDE_FILT_ZETA_R * STABILIZATION_ATTITUDE_FILT_OMEGA_R 
  //                     + ( stateGetBodyRates_f()->r - filt_ang_rate.r ) * STABILIZATION_ATTITUDE_FILT_OMEGA_SQR_R;
}

static void stabilization_inputs_filter(void)
{
u_act.p = ( -rpm_motor[0] + rpm_motor[1] + rpm_motor[2] - rpm_motor[3] ) * MOTOR_SCALING_VALUE;
u_act.q = (  rpm_motor[0] + rpm_motor[1] - rpm_motor[2] - rpm_motor[3] ) * MOTOR_SCALING_VALUE;
u_act.r = (  rpm_motor[0] - rpm_motor[1] + rpm_motor[2] - rpm_motor[3] ) * MOTOR_SCALING_VALUE;

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
// filt_u_act_ddot.r = -filt_u_act_dot.r * 2  * STABILIZATION_ATTITUDE_FILT_ZETA_R * STABILIZATION_ATTITUDE_FILT_OMEGA_R
//                   + (u_act.r - filt_u_act.r) * STABILIZATION_ATTITUDE_FILT_OMEGA_SQR_R;
}

static void attitude_run_smc(int32_t smc_commands[])
{

  memcpy(rpm_motor, actuators_bebop.rpm_obs, 4*sizeof(uint16_t));

  h.p = stabilization_attitude_lambda_0.x * filt_ang_rate.p
      + stabilization_attitude_lambda_1.x * filt_ang_rate_dot.p;
  h.q = stabilization_attitude_lambda_0.y * filt_ang_rate.q
      + stabilization_attitude_lambda_1.y * filt_ang_rate_dot.q;
  // h.r = stabilization_attitude_lambda_0.z * filt_ang_rate.r
  //     + stabilization_attitude_lambda_1.z * filt_ang_rate_dot.r;

  s_func.p = stabilization_attitude_lambda_0.x * -1 * QUAT1_FLOAT_OF_BFP(att_err.qx)
           + stabilization_attitude_lambda_1.x * -1 * RATE_FLOAT_OF_BFP(rate_err.p);
  s_func.q = stabilization_attitude_lambda_0.y * -1 * QUAT1_FLOAT_OF_BFP(att_err.qy)
           + stabilization_attitude_lambda_1.y * -1 * RATE_FLOAT_OF_BFP(rate_err.q);
  // s_func.r = stabilization_attitude_lambda_0.z * -1 * QUAT1_FLOAT_OF_BFP(att_err.qz)
  //          + stabilization_attitude_lambda_1.y * -1 * RATE_FLOAT_OF_BFP(rate_err.r);

  s_contr_incr.p = -(h.p + stabilization_attitude_k.x * stabilization_smc_sign(s_func.p)) 
                  * stabilization_attitude_inv_input_distr.p;
  s_contr_incr.q = -(h.q + stabilization_attitude_k.y * stabilization_smc_sign(s_func.q))
                  * stabilization_attitude_inv_input_distr.q;
  // s_contr_incr.r = -(h.r + stabilization_attitude_k.z * stabilization_smc_sign(s_func.r))
  //                 * STABILIZATION_ATTITUDE_INV_INPUT_DISTR_R;

  stabilization_inputs_filter();

  u.p = filt_u_act.p + s_contr_incr.p;
  u.q = filt_u_act.q + s_contr_incr.q;
  // u.r = filt_u_act.r + s_contr_incr.r;

  u.r = GAIN_PRESCALER_P * STABILIZATION_ATTITUDE_PSI_PGAIN * QUAT1_FLOAT_OF_BFP(att_err.qz) / 4 +
        GAIN_PRESCALER_D * STABILIZATION_ATTITUDE_PSI_DGAIN  * RATE_FLOAT_OF_BFP(rate_err.r)  / 16;

  BOUND_CONTROLS(u.p, -4500., 4500.);
  BOUND_CONTROLS(u.q, -4500., 4500.);
  float half_thrust = ((float) stabilization_cmd[COMMAND_THRUST]/2);
  BOUND_CONTROLS(u.r, -half_thrust, half_thrust);

  //Don't increment if thrust is off
  if(stabilization_cmd[COMMAND_THRUST]<300) {
    FLOAT_RATES_ZERO(s_contr_incr);
    FLOAT_RATES_ZERO(u);
  }

  /*  SMC feedback */
  smc_commands[COMMAND_ROLL] = u.p;
  smc_commands[COMMAND_PITCH] = u.q;
  smc_commands[COMMAND_YAW] = u.r;
}

void stabilization_attitude_run(bool_t enable_integrator) 
{
  /* Update reference */
  stabilization_attitude_ref_update();

  /* Propagate the second order filter on the gyroscopes */
  stabilization_rate_filter();

  /* attitude error */
  att_quat = stateGetNedToBodyQuat_i();
  INT32_QUAT_INV_COMP(att_err, *att_quat, stab_att_ref_quat);

  /* wrap it in the shortest direction */
  INT32_QUAT_WRAP_SHORTEST(att_err);
  INT32_QUAT_NORMALIZE(att_err);

  /*  rate error */
  const struct Int32Rates rate_ref_scaled = {
    OFFSET_AND_ROUND(stab_att_ref_rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC))
  };
  struct Int32Rates *body_rate = stateGetBodyRates_i();
  RATES_DIFF(rate_err, rate_ref_scaled, (*body_rate));

  /* compute the INDI command */
  attitude_run_smc(stabilization_att_smc_cmd);

  stabilization_cmd[COMMAND_ROLL] = stabilization_att_smc_cmd[COMMAND_ROLL];
  stabilization_cmd[COMMAND_PITCH] = stabilization_att_smc_cmd[COMMAND_PITCH];
  stabilization_cmd[COMMAND_YAW] = stabilization_att_smc_cmd[COMMAND_YAW];

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
}

void stabilization_attitude_read_rc(bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn) {
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}