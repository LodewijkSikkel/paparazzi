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

#ifndef STABILIZATION_ATTITUDE_QUAT_SMC_H
#define STABILIZATION_ATTITUDE_QUAT_SMC_H

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

#include "math/pprz_algebra_int.h"
struct ReferenceSystem {
  float err_p;
  float err_q;
  float err_r;
  float rate_p;
  float rate_q;
  float rate_r;
};

extern struct ReferenceSystem reference_acceleration;

extern struct FloatRates filt_ang_rate;
extern struct FloatRates filt_ang_rate_dot;
extern struct FloatRates filt_ang_rate_ddot;

extern uint16_t rpm_motor[4]; 
extern struct FloatVect3 smc_delta_rpm;

extern struct FloatVect3 lambda_0;
extern struct FloatVect3 lambda_1;
extern struct FloatVect3 K;

#endif /* STABILIZATION_ATTITUDE_QUAT_SMC_H */