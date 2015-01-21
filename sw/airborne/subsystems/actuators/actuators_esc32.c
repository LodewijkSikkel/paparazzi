/*
 * Copyright (C) 2014 Lodewijk Sikkel <l.n.c.sikkel@tudelft.nl>
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

/** @file actuators_esc32.c
 *  Actuator driver for AutoQuad ESC32 motor controllers.
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_esc32.h"

#include "led.h"

#include "mcu.h"
#include "mcu_periph/sys_time.h"

#include "autopilot.h"

uint8_t esc32_state[MOTORS_NUM] = {0, 0, 0, 0};

esc32_struct_t esc32_data;

static void esc32_can_send_groups(void) 
{
    int i;

    for (i = 0; i < esc32_data.num_groups; i++)
        can_command_setpoint16(i+1, (uint8_t *)&esc32_data.can_groups[i]);
}

static void esc32_can_request_telem(int motor_ID) 
{
#if ESC32_CAN_TELEM_RATE > 0
  // request telemetry
  can_set_telem_value(ESC32_CAN_TT_NODE, esc32_data.can[motor_id]->network_id, 0, ESC32_CAN_TELEM_STATUS);

  // set telemetry rate
  can_set_telem_rate(ESC32_CAN_TT_NODE, esc32_data.can[motor_id]->network_id, ESC32_CAN_TELEM_RATE);
#endif
}

static void esc32_actuators_can_init(int i) 
{
  uint8_t num_try = MOTORS_CAN_RETRIES;

  while (num_try-- && (esc32_data.can[i] = can_find_node(CAN_TYPE_ESC)) == 0)
      sys_time_usleep(100000); // wait 100ms
}

void esc32_init() 
{
  actuators_can_init();

  esc32_data.num_groups = 1;

  int i;

  for (i = 0; i < MOTORS_NUM; i++)
    esc32_actuators_can_init(i);

  // register the telemetry callback function
  can_telem_register(esc32_receive_telem, CAN_TYPE_ESC);
}

int esc32_arm(void) 
{
    int tries = 1;
    int i;

    // group arm
    for (i = 0; i < esc32_data.num_groups; i++)
        can_command_arm(CAN_TT_GROUP, i+1);

    // wait for all motors to arm
    for (i = 0; i < MOTORS_NUM; i++)
        if (esc32_data.can[i]) {
            tries = 3;
            while (--tries && (esc32_state[i] = *can_get_state(esc32_data.can[i]->network_id)) == ESC32_STATE_DISARMED) 
            {
              sys_time_usleep(1000); // wait 1ms
            }

            if (!tries)
              break;
        }

    return tries;
}

void esc32_disarm(void) 
{
    int i;

    // group disarm
    for (i = 0; i < esc32_data.num_groups; i++)
        can_command_disarm(CAN_TT_GROUP, i+1);
}

bool temp = false;

void esc32_commit(void) 
{
  int i;

  for (i = 0; i < esc32_data.num_groups; i++) 
  {
    can_command_setpoint16(1, (uint8_t *) esc32_data.cmds);
    
    if (autopilot_motors_on) {
      can_command_arm(CAN_TT_GROUP, i+1);
    }
  }
}


void esc32_set(uint8_t i, uint16_t v)
{
  esc32_data.cmds[i] = v << 4; 
}

void esc32_config_cmd(uint8_t i)
{
}

void esc32_receive_telem(uint8_t can_id, uint8_t doc, void *data_ptr) {
  uint32_t *data = (uint32_t *)data_ptr;
  uint32_t *storage = (uint32_t *)&esc32_data.can_status[can_id-1];

  // copy status data to our storage (8 bytes)
  storage[0] = data[0];
  storage[1] = data[1];
}

