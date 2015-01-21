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
 * @file actuator_can_driver.h
 *
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_can.h"

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/can.h"

can_struct_t actuators_can_data;

static uint32_t can_get_seq_id(void) {
    uint32_t seq_id;

    seq_id = actuators_can_data.seq_id;
    actuators_can_data.seq_id = (actuators_can_data.seq_id + 1) & 0x3f;

    return seq_id;
}

static void can_grant_addr(uint8_t *data) {
  // the first byte of the received message contains the uuid
  uint32_t *uuid_ptr = (uint32_t*)data;
  uint32_t uuid;
  uint8_t i;

  uuid = *uuid_ptr;

  // look for this uuid in our address table
  for (i = 0; i < actuators_can_data.next_node_slot; i++)
      if (actuators_can_data.nodes[i].uuid == uuid)
          break;

  if (i == actuators_can_data.next_node_slot)
      actuators_can_data.next_node_slot++;

  if (i < (CAN_t_id_MASK>>9)) {
      // store in table
      actuators_can_data.nodes[i].network_id = i+1;
      actuators_can_data.nodes[i].uuid = uuid;
      actuators_can_data.nodes[i].type = data[4];
      actuators_can_data.nodes[i].can_id = data[5];

      // send group ID & subgroup ID in bytes 5 & 6
      data[4] = (i / 4) + 1; // group ID
      data[5] = (i % 4) + 1; // subgroup ID

      // respond
      can_send(CAN_LCC_HIGH | CAN_TT_NODE | CAN_FID_GRANT_ADDR, i+1, 6, data);
  }
}

static uint8_t *can_send_wait_response(uint32_t ext_id, uint8_t t_id, uint8_t n, uint8_t *data) {
    int16_t seq_id;
    int timeout = CAN_TIMEOUT;

    seq_id = can_send(ext_id, t_id, n, data);

    do {
        sys_time_usleep(1000); // wait 1ms
        timeout--;
    } while (timeout && actuators_can_data.responses[seq_id] == 0);

    if (timeout == 0) {
        actuators_can_data.timeouts++;
        return 0;
    } else if (actuators_can_data.responses[seq_id] != (CAN_FID_NACK>>25)) {
        return &actuators_can_data.response_data[seq_id*8];
    } else { // NACK
        return 0;
    }
}

void actuators_can_init(void)
{
  // initialize the CAN bus
  ppz_can_init(actuators_can_rx_callback);

  // reset the CAN bus
  can_reset_bus();
}

uint8_t can_send(uint32_t id, uint8_t t_id, uint8_t length, uint8_t *data)
{
  uint8_t seq_id = can_get_seq_id();

  ppz_can_transmit((id >> 3) | ((t_id & 0x1f) << 6) | seq_id, data, length);
  
  return seq_id;
}

void actuators_can_rx_callback(uint32_t id, uint8_t *data, int len __attribute__((unused)))
{
  // deconstruct the message ID
  uint8_t doc = ((id<<3) & CAN_DOC_MASK)>>19;
  uint8_t sid = ((id<<3) & CAN_SID_MASK)>>14;

  switch ((id<<3) & CAN_FID_MASK) {
      case CAN_FID_REQ_ADDR:
          can_grant_addr(data);
          break;

      // telemetry callbacks
      case CAN_FID_TELEM:
            if (actuators_can_data.telem_funcs[actuators_can_data.nodes[sid-1].type])
                actuators_can_data.telem_funcs[actuators_can_data.nodes[sid-1].type](actuators_can_data.nodes[sid-1].can_id, doc, data);
            break;

      case CAN_FID_CMD:
          // do nothing
      case CAN_FID_ACK:
          // do nothing
      case CAN_FID_NACK:
          // do nothing
      case CAN_FID_REPLY:
          // do nothing
      default:
          // do nothing
          break;
  }
}

void can_command_beep(uint32_t tt, uint8_t t_id, uint16_t freq, uint16_t dur) {
  uint16_t data[2];

  data[0] = freq;
  data[1] = dur;

  can_send(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_BEEP<<19), t_id, 4, (uint8_t *)&data);
}

uint8_t *can_get_state(uint8_t t_id) {
  return (uint8_t *)can_send_wait_response(CAN_LCC_NORMAL | CAN_TT_NODE | CAN_FID_GET | (actuators_can_data_STATE<<19), t_id, 0, 0);
}

void can_command_arm(uint32_t tt, uint8_t t_id) {
  can_send(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_ARM<<19), t_id, 0, 0);
}

void can_command_disarm(uint32_t tt, uint8_t t_id) {
  can_send(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_DISARM<<19), t_id, 0, 0);
}

can_nodes_t *can_find_node(uint8_t type) {
  uint8_t i;

  for (i = 0; i <= (CAN_t_id_MASK>>9); i++)
      if (actuators_can_data.nodes[i].type == type)
          return &actuators_can_data.nodes[i];

  return 0;
}

void can_set_telem_value(uint32_t tt, uint8_t t_id, uint8_t index, uint8_t value) {
  uint8_t data[2];

  data[0] = index;
  data[1] = value;

  can_send(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_TELEM_VALUE<<19), t_id, 2, data);
}

void can_set_telem_rate(uint32_t tt, uint8_t t_id, uint16_t rate) {
  uint16_t data;

  data = rate;

  can_send(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_TELEM_RATE<<19), t_id, 2, (uint8_t *)&data);
}

void can_command_start(uint32_t tt, uint8_t t_id) {
  can_send(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_START << 19), t_id, 0, 0);
}

uint8_t *can_command_stop(uint32_t tt, uint8_t t_id) {
    return can_send_wait_response(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_STOP<<19), t_id, 0, 0);
}

void can_reset_bus(void) {
  can_send(CAN_LCC_EXCEPTION | CAN_TT_GROUP | CAN_FID_RESET_BUS, 0, 0, 0);
}

void can_telem_register(can_telem_callback_t *func, uint8_t type) {
  actuators_can_data.telem_funcs[type] = func;
}

void can_command_setpoint16(uint8_t t_id, uint8_t *data) {
  can_send(CAN_LCC_HIGH | CAN_TT_GROUP | CAN_FID_CMD | (CAN_CMD_SETPOINT16<<19), t_id, 8, data);
}