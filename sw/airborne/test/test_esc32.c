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
 * @file test_esc32.c
 *
 */

#define NB_ACTUATORS 1

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/can.h"

#include <std.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Logical Communications Channel
// 2 bits [28:27]
#define ESC32_CAN_LCC_MASK            ((uint32_t)0x3<<30)
#define ESC32_CAN_LCC_EXCEPTION       ((uint32_t)0x0<<30)
#define ESC32_CAN_LCC_HIGH            ((uint32_t)0x1<<30)
#define ESC32_CAN_LCC_NORMAL          ((uint32_t)0x2<<30)
#define ESC32_CAN_LCC_INFO            ((uint32_t)0x3<<30)

// Target Type
// 1 bit [26:26]
#define ESC32_CAN_TT_MASK             ((uint32_t)0x1<<29)
#define ESC32_CAN_TT_GROUP            ((uint32_t)0x0<<29)
#define ESC32_CAN_TT_NODE             ((uint32_t)0x1<<29)

// Function ID
// 4 bits [25:22]
#define ESC32_CAN_FID_MASK            ((uint32_t)0xf<<25)
#define ESC32_CAN_FID_RESET_BUS       ((uint32_t)0x0<<25)
#define ESC32_CAN_FID_ACK             ((uint32_t)0x1<<25)
#define ESC32_CAN_FID_NACK            ((uint32_t)0x2<<25)
#define ESC32_CAN_FID_CMD             ((uint32_t)0x3<<25)
#define ESC32_CAN_FID_GET             ((uint32_t)0x4<<25)
#define ESC32_CAN_FID_SET             ((uint32_t)0x5<<25)
#define ESC32_CAN_FID_REPLY           ((uint32_t)0x6<<25)
#define ESC32_CAN_FID_REQ_ADDR        ((uint32_t)0x7<<25)
#define ESC32_CAN_FID_GRANT_ADDR      ((uint32_t)0x8<<25)
#define ESC32_CAN_FID_ERROR           ((uint32_t)0x9<<25)
#define ESC32_CAN_FID_PING            ((uint32_t)0xa<<25)
#define ESC32_CAN_FID_TELEM           ((uint32_t)0xb<<25)

// Data Object Code
// 6 bits [21:16]
#define ESC32_CAN_DOC_MASK            ((uint32_t)0x3f<<19)

// Source ID
// 5 bits [15:11]
#define ESC32_CAN_SID_MASK            ((uint32_t)0x1f<<14)

// Target ID
// 5 bits [10:6]
#define ESC32_CAN_TID_MASK            ((uint32_t)0x1f<<9)

// Sequence ID
// 6 bits [5:0]
#define ESC32_CAN_SEQ_MASK            ((uint32_t)0x3f<<3)

// types
enum {
  ESC32_CAN_TYPE_ESC = 1,
  ESC32_CAN_TYPE_SERVO,
  ESC32_CAN_TYPE_SENSOR,
  ESC32_CAN_TYPE_SWITCH,
  ESC32_CAN_TYPE_OSD,
  ESC32_CAN_TYPE_UART,
  ESC32_CAN_TYPE_HUB,
  ESC32_CAN_TYPE_NUM
};

// commands
enum {
  ESC32_CAN_CMD_DISARM = 1,
  ESC32_CAN_CMD_ARM,
  ESC32_CAN_CMD_START,
  ESC32_CAN_CMD_STOP,
  ESC32_CAN_CMD_SETPOINT10,
  ESC32_CAN_CMD_SETPOINT12,
  ESC32_CAN_CMD_SETPOINT16,
  ESC32_CAN_CMD_RPM,
  ESC32_CAN_CMD_CFG_READ,
  ESC32_CAN_CMD_CFG_WRITE,
  ESC32_CAN_CMD_CFG_DEFAULT,
  ESC32_CAN_CMD_TELEM_RATE,
  ESC32_CAN_CMD_TELEM_VALUE,
  ESC32_CAN_CMD_BEEP,
  ESC32_CAN_CMD_POS,
  ESC32_CAN_CMD_USER_DEFINED,
  ESC32_CAN_CMD_RESET,
  ESC32_CAN_CMD_STREAM,
  ESC32_CAN_CMD_ON,
  ESC32_CAN_CMD_OFF
};

// data types
enum {
  ESC32_CAN_DATA_GROUP = 1,
  ESC32_CAN_DATA_TYPE,
  ESC32_CAN_DATA_ID,
  ESC32_CAN_DATA_INPUT_MODE,
  ESC32_CAN_DATA_RUN_MODE,
  ESC32_CAN_DATA_STATE,
  ESC32_CAN_DATA_PARAM_ID,
  ESC32_CAN_DATA_TELEM,
  ESC32_CAN_DATA_VERSION,
  ESC32_CAN_DATA_VALUE,
  ESC32_CAN_DATA_PARAM_NAME1,
  ESC32_CAN_DATA_PARAM_NAME2
};

// telemetry values
enum {
  ESC32_CAN_TELEM_NONE = 0,
  ESC32_CAN_TELEM_STATUS,
  ESC32_CAN_TELEM_STATE,
  ESC32_CAN_TELEM_TEMP,
  ESC32_CAN_TELEM_VIN,
  ESC32_CAN_TELEM_AMPS,
  ESC32_CAN_TELEM_RPM,
  ESC32_CAN_TELEM_ERRORS,
  ESC32_CAN_TELEM_VALUE,
  ESC32_CAN_TELEM_NUM
};

typedef struct {
    uint32_t uuid;
    uint8_t network_ID;
    uint8_t type;
    uint8_t can_ID;
    uint8_t group_ID;
    uint8_t subgroup_ID;
} can_nodes_t;

enum ESC32_status {
  ESC32_STATUS_INIT = 0,
  ESC32_STATUS_MELODY,
  ESC32_STATUS_UNARMED,
  ESC32_STATUS_RUNNING
};

typedef struct {
    enum ESC32_status status;
    can_nodes_t nodes[(ESC32_CAN_TID_MASK>>9)+1];
    uint8_t next_node_slot;
    uint8_t seq_ID;
} can_struct_t;

can_struct_t can_data;

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);

void esc32_can_init(void);
void esc32_can_command_beep(uint32_t tt, uint8_t tid, uint16_t freq, uint16_t dur);
void esc32_can_command_arm(uint32_t tt, uint8_t tid);
void esc32_can_command_disarm(uint32_t tt, uint8_t tid);
void esc32_can_start(uint32_t tt, uint8_t tid);
void esc32_can_reset_bus(void);

uint8_t esc32_can_send(uint32_t ID, uint8_t t_ID, uint8_t length, uint8_t *data);

void esc32_can_rx_cb(uint32_t ID, uint8_t *data, int len);

static uint32_t esc32_can_get_seq_id(void) {
    uint32_t seq_ID;

    seq_ID = can_data.seq_ID;
    can_data.seq_ID = (can_data.seq_ID + 1) & 0x3f;

    return seq_ID;
}

void esc32_can_init(void)
{
  // initialize the CAN bus
  ppz_can_init(esc32_can_rx_cb);

  // reset the CAN bus
  esc32_can_reset_bus();

  can_data.status = ESC32_STATUS_UNARMED;
}

void esc32_can_command_beep(uint32_t tt, uint8_t tid, uint16_t freq, uint16_t dur) {
    uint16_t data[2];

    data[0] = freq;
    data[1] = dur;

    return esc32_can_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_CMD | (ESC32_CAN_CMD_BEEP<<19), tid, 4, (uint8_t *)&data);
}

void esc32_can_command_arm(uint32_t tt, uint8_t tid) {
  esc32_can_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_CMD | (ESC32_CAN_CMD_ARM<<19), tid, 0, 0);
}

void esc32_can_command_disarm(uint32_t tt, uint8_t tid) {
  esc32_can_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_CMD | (ESC32_CAN_CMD_DISARM<<19), tid, 0, 0);
}

void esc32_can_start(uint32_t tt, uint8_t tid) {
  esc32_can_send(ESC32_CAN_LCC_NORMAL | tt | ESC32_CAN_FID_CMD | (ESC32_CAN_CMD_START << 19), tid, 0, 0);
}

void esc32_can_reset_bus(void) {
  esc32_can_send(ESC32_CAN_LCC_EXCEPTION | ESC32_CAN_TT_GROUP | ESC32_CAN_FID_RESET_BUS, 0, 0, 0);
}

uint8_t esc32_can_send(uint32_t ID, uint8_t t_ID, uint8_t length, uint8_t *data)
{
  uint8_t seq_ID = 0;

  ppz_can_transmit((ID >> 3) | ((t_ID & 0x1f) << 6) | seq_ID, data, length);
  
  return 0;
}

static void esc32_can_grant_addr(uint8_t *data) {
  // the first byte of the received message contains the uuid
  uint32_t *uuid_ptr = (uint32_t*)data;
  uint32_t uuid;
  uint8_t i;

  uuid = *uuid_ptr;

  // look for this uuid in our address table
  for (i = 0; i < can_data.next_node_slot; i++)
      if (can_data.nodes[i].uuid == uuid)
          break;

  if (i == can_data.next_node_slot)
      can_data.next_node_slot++;

  if (i < (ESC32_CAN_TID_MASK>>9)) {
      // store in table
      can_data.nodes[i].network_ID = i+1;
      can_data.nodes[i].uuid = uuid;
      can_data.nodes[i].type = data[4];
      can_data.nodes[i].can_ID = data[5];

      // send groupId & subgroupId in bytes 5 & 6
      data[4] = (i / 4) + 1; //GroupId
      data[5] = (i % 4) + 1; //SubGroupId

      // respond
      esc32_can_send(ESC32_CAN_LCC_HIGH | ESC32_CAN_TT_NODE | ESC32_CAN_FID_GRANT_ADDR, i+1, 6, data);
  }
}

void esc32_can_rx_cb(uint32_t id, uint8_t *data, int len __attribute__((unused)))
{
  // deconstruct the message ID
  //uint8_t doc = ((id<<3) & ESC32_CAN_DOC_MASK)>>19;
  //uint8_t sid = ((id<<3) & ESC32_CAN_SID_MASK)>>14;
  //uint8_t seq_ID = ((id<<3) & ESC32_CAN_SEQ_MASK)>>3;

  switch ((id<<3) & ESC32_CAN_FID_MASK) {
      case ESC32_CAN_FID_REQ_ADDR:
          esc32_can_grant_addr(data);
          break;

      // telemetry callbacks
      case ESC32_CAN_FID_TELEM:
          // do nothing
      case ESC32_CAN_FID_CMD:
          // do nothing
      case ESC32_CAN_FID_ACK:
          // do nothing
      case ESC32_CAN_FID_NACK:
          // do nothing
      case ESC32_CAN_FID_REPLY:
          // do nothing
      default:
          // do nothing
          break;
  }
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
  sys_time_register_timer((0.5 / PERIODIC_FREQUENCY), NULL);

  sys_time_usleep(3000000);

  // initialize the can_data struct
  can_data.status = ESC32_STATUS_INIT;
  can_data.next_node_slot = 0;

  // initialize the ESC32 group
  esc32_can_init();

  // wait 100ms until all nodes are registered
  sys_time_usleep(100000);

  // arm the group
  if (can_data.status == ESC32_STATUS_UNARMED) {
    esc32_can_command_arm(ESC32_CAN_TT_GROUP, 1);
    can_data.status = ESC32_STATUS_MELODY;
  }
  // start the group
  if (can_data.status == ESC32_STATUS_MELODY) {
    esc32_can_start(ESC32_CAN_TT_GROUP, 1);
    can_data.status = ESC32_STATUS_RUNNING;
  }
}

static inline void main_periodic_task(void)
{
}

static inline void main_event_task(void)
{
  // commence a beep 
  if (can_data.status == ESC32_STATUS_RUNNING)
    esc32_can_command_beep(ESC32_CAN_TT_GROUP, 1, 600, 100);
}
