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
 *
 */

/** @file subsystems/intermcu/intermcu_standalone.c
 *  @brief Inter-MCU communication protocol for standalone applications
 */

#include "intermcu_standalone.h"
#include "pprzlink/intermcu_msg.h"
#include "pprzlink/pprz_transport.h"
#include "mcu_periph/uart.h"

#if INTERMCU_AP
#include "subsystems/imu.h"
#endif

#if INTERMCU_AP
#include "subsystems/gps.h"
#endif

#if INTERMCU_EKF
#include "standalone/extended_kalman_filter/ekf.h"
#endif

// Used for communication
static struct link_device *intermcu_device = (&((INTERMCU_LINK).device));
static struct pprz_transport intermcu_transport;

struct intermcu_t inter_mcu;
static inline void intermcu_parse_msg(struct transport_rx *trans, void (*commands_frame_handler)(void));


void intermcu_init(void)
{
  pprz_transport_init(&intermcu_transport);
}

void intermcu_periodic(void)
{
  /* Check for interMCU loss */
  if (inter_mcu.time_since_last_frame >= INTERMCU_LOST_CNT) {
    inter_mcu.status = INTERMCU_LOST;
  } else {
    inter_mcu.time_since_last_frame++;
  }
}

void intermcu_send(void)
{
#if INTERMCU_AP
  // Compute the ground speed in the North-East Down Earth reference frame
  struct NedCoor_i gps_ned_vel_i;
  ned_of_ecef_vect_i(&gps_ned_vel_i, &state.ned_origin_i, &gps.ecef_vel);

  // Compute the ground speed in the body reference frame
  struct FloatVect3 gps_body_vel_i;
  int32_rmat_vmult(&gps_body_vel_i, stateGetNedToBodyRMat_i(), &gps_ned_vel_i);

  pprz_msg_send_IMCU_SENSOR_MEASUREMENTS(&(intermcu_transport.trans_tx), intermcu_device, INTERMCU_STANDALONE, 
                                         &imu.gyro.p, &imu.gyro.q, &imu.gyro.r,
                                         &imu.accel.x, &imu.accel.y, &imu.accel.z,
                                         &gps_body_vel_i.x, &gps_body_vel_i.y, &gps_body_vel_i.z); 
#else

#endif
}

static inline void intermcu_parse_msg(struct transport_rx *trans, void (*commands_frame_handler)(void))
{
  /* Parse the Inter MCU message */
  uint8_t msg_id = trans->payload[1];
  switch (msg_id) {
#if INTERMCU_EKF
    case DL_IMCU_SENSOR_MEASUREMENTS: {
      LED_TOGGLE(3);

      parse_sensor_measurements(DL_IMCU_SENSOR_MEASUREMENTS_gyro_p(payload), 
                                DL_IMCU_SENSOR_MEASUREMENTS_gyro_q(payload), 
                                DL_IMCU_SENSOR_MEASUREMENTS_gyro_r(payload),
                                DL_IMCU_SENSOR_MEASUREMENTS_accel_x(payload),
                                DL_IMCU_SENSOR_MEASUREMENTS_accel_y(payload),
                                DL_IMCU_SENSOR_MEASUREMENTS_accel_z(payload),
                                DL_IMCU_SENSOR_MEASUREMENTS_gps_body_vel_x(payload),
                                DL_IMCU_SENSOR_MEASUREMENTS_gps_body_vel_y(payload),
                                DL_IMCU_SENSOR_MEASUREMENTS_gps_body_vel_z(payload));
      inter_mcu.status = INTERMCU_OK;
      inter_mcu.time_since_last_frame = 0;
      commands_frame_handler(); // handles the incoming data
      break;
    }
#endif
    default:
      break;
  }

  // Set to receive another message
  trans->msg_received = FALSE;
}

void InterMcuEvent(void (*frame_handler)(void))
{
  /* Parse incoming bytes */
  if (intermcu_device->char_available(intermcu_device->periph)) {
    while (intermcu_device->char_available(intermcu_device->periph) && !intermcu_transport.trans_rx.msg_received) {
      parse_pprz(&intermcu_transport, intermcu_device->get_byte(intermcu_device->periph));
    }

    if (intermcu_transport.trans_rx.msg_received) {
      intermcu_parse_msg(&(intermcu_transport.trans_rx), frame_handler);
    }
  }
}
