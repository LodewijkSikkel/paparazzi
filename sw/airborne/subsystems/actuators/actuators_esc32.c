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

motors_struct_t motors_data;

void esc32_receive_telem(uint8_t can_id, uint8_t doc, void *data_ptr) {
  uint32_t *data = (uint32_t *)data_ptr;
  uint32_t *storage = (uint32_t *)&motors_data.can_status[can_id];
  uint32_t micros = get_sys_time_usec();

  // copy status data to our storage (8 bytes)
  storage[0] = data[0];
  storage[1] = data[1];

  // record reception time
  motors_data.can_status_time[can_id] = micros;
}

static void esc32_can_request_telem(int motor_id) 
{
#if MOTORS_CAN_TELEM_RATE > 0
  // request telemetry
  can_set_telem_value_no_wait(CAN_TT_NODE, motors_data.can[motor_id]->network_id, 0, CAN_TELEM_STATUS);

  // set telemetry rate
  can_set_telem_rate_no_wait(CAN_TT_NODE, motors_data.can[motor_id]->network_id, MOTORS_CAN_TELEM_RATE);

  motors_data.can_telem_req_time[motor_id] = get_sys_time_usec();
#endif
}

void esc32_can_send_groups(void) 
{
    // int i;

    // for (i = 0; i < motors_data.num_groups; i++)
        // can_command_setpoint16(i+1, (uint8_t *)&motors_data.can_groups[i]);
}

static void esc32_check_can_status(int motor_id) {
#if MOTORS_CAN_TELEM_RATE > 0
  // no status report within the last second?
  if ((get_sys_time_usec() - motors_data.can_status_time[motor_id]) > 1e6f) 
    {
    // has it been more than 1 second since our last request?
    if ((get_sys_time_usec() - motors_data.can_telem_req_time[motor_id]) > 1e6f) 
    {
      // clear status information
      uint32_t *storage = (uint32_t *)&motors_data.can_status[motor_id];
      storage[0] = 0;
      storage[1] = 0;

      LED_TOGGLE(3);

      esc32_can_request_telem(motor_id);
    }
  }
//   // if ESC is reporting as being disarmed (and should not be)
//   else 
    // if (motors_data.can_status[motor_id].state == ESC32_STATE_DISARMED && autopilot_motors_on) {
    // // send an arm command
    // can_command_arm(CAN_TT_NODE, motors_data.can[motor_id]->network_id);
  // }
#endif
}

void esc32_send_values(uint8_t motor_id, uint16_t value) {
  esc32_check_can_status(motor_id);

  // if (autopilot_motors_on) 
  //     *motors_data.can_ptrs[motor_id] = value << 4;
  // else
  //     *motors_data.can_ptrs[motor_id] = 0;
}

int esc32_arm(void) 
{
    int tries = 1;
    int i;

    // group arm
    for (i = 0; i < motors_data.num_groups; i++)
        can_command_arm(CAN_TT_GROUP, i+1);

    // wait for all motors to arm
    for (i = 0; i < MOTORS_NUM; i++)
        if (motors_data.can[i]) {
            tries = 3;
            while (--tries && *can_get_state(motors_data.can[i]->network_id) == ESC32_STATE_DISARMED) 
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
    for (i = 0; i < motors_data.num_groups; i++)
        can_command_disarm(CAN_TT_GROUP, i+1);
}

static void esc32_actuators_can_init(int i) 
{
  uint8_t num_try = MOTORS_CAN_RETRIES;
  uint8_t motor_id = i;

  while (num_try-- && (motors_data.can[motor_id] = can_find_node(CAN_TYPE_ESC, i+1)) == 0)
      sys_time_usleep(100000); // wait 100ms

  // if (motors_data.can[motor_id] != 0) 
    //esc32_can_request_telem(motor_id);
}

static void esc32_set_can_group(void) {
    int group;
    int subgroup;
    int i;

    group = 0;
    subgroup = 0;
    for (i = 0; i < MOTORS_NUM; i++) {
      if (motors_data.can[i]) {
          can_set_group(motors_data.can[i]->network_id, group+1, subgroup+1);

          switch (subgroup) {
        case 0:
            motors_data.can_ptrs[i] = &motors_data.can_groups[group].value1;
            motors_data.num_groups++;
            break;
        case 1:
            motors_data.can_ptrs[i] = &motors_data.can_groups[group].value2;
            break;
        case 2:
            motors_data.can_ptrs[i] = &motors_data.can_groups[group].value3;
            break;
        case 3:
            motors_data.can_ptrs[i] = &motors_data.can_groups[group].value4;
            break;
          }

          subgroup++;
          if (subgroup == MOTORS_CAN_GROUP_SIZE) {
        group++;
        subgroup = 0;
          }
      }
    }
}

void esc32_init() 
{
  init_can();

  // fill the motors_data struct with zeros
  memset((void *)&motors_data, 0, sizeof(motors_data));

  int i;

  // initialize the escs
  for (i = 2; i < MOTORS_NUM; i++)
    esc32_actuators_can_init(i);

  // register the telemetry callback function
  can_telem_register(esc32_receive_telem, CAN_TYPE_ESC);

  // set the initial command
  esc32_set_can_group();
}