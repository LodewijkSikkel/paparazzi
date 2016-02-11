/*
 * Copyright (C) 20015 Lodewijk Sikkel <l.n.c.sikkel>
 *
 * This file is a standalone program running a Extended 
 * Kalman Filter (EKF). It receives 
 */

#include <inttypes.h>

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"
#include "led.h"
#include "subsystems/intermcu/intermcu_standalone.h"
#include "standalone/extended_kalman_filter/ekf.h"

#define PACKED __attribute__((__packed__))

struct PACKED log_uart_data_struct {
  uint8_t start_byte;  // 1
  uint8_t length;      // 2
  // uint16_t timestamp;  // 4
  int16_t gyro_p;      // 6
  int16_t gyro_q;      // 8
  int16_t gyro_r;      // 10
  int16_t accel_x;     // 12
  int16_t accel_y;     // 14
  int16_t accel_z;     // 16
  int16_t gps_body_x;  // 18
  int16_t gps_body_y;  // 20
  int16_t gps_body_z;  // 22
}; // struct of 22 bytes

struct log_uart_data_struct log_uart_data;

static void send_log(void) 
{
  log_uart_data.start_byte = 0x99;
  log_uart_data.length = 20; // bytes

  // uint32_t time_in_milli_seconds = get_sys_time_msec() / 10;
  // log_uart_data.timestamp = (uint16_t)time_in_milli_seconds; // in seconds

  log_uart_data.gyro_p = sensors.gyro_i.p;
  log_uart_data.gyro_q = sensors.gyro_i.q;
  log_uart_data.gyro_r = sensors.gyro_i.r;
  log_uart_data.accel_x = sensors.accel_i.x;
  log_uart_data.accel_y = sensors.accel_i.y;
  log_uart_data.accel_z = sensors.accel_i.z;
  log_uart_data.gps_body_x = sensors.gps_body_vel_i.x;
  log_uart_data.gps_body_y = sensors.gps_body_vel_i.y;
  log_uart_data.gps_body_z = sensors.gps_body_vel_i.z;

  uint8_t crc = 0;
  uint8_t *bytes = (uint8_t*) &log_uart_data;
  for (int i = 0; i < log_uart_data.length; i++)
  {
    crc += bytes[i];
    uart_put_byte(&LOGGER_PORT, bytes[i]);
  }
  uart_put_byte(&LOGGER_PORT, crc);

  RunOnceEvery(10, {
    LED_TOGGLE(5);
  });
}

static void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);

  intermcu_init(); // initialize the inter-mcu connection

  ekf_init();
}

static void main_periodic_task(void)
{ 
  LED_TOGGLE(2);
  intermcu_periodic(); // check the connection 

  // ekf_periodic();

  intermcu_send(); // send the data frame

  send_log();
}

static void handle_incoming_frame(void) // deprecated
{
}

static void main_event_task(void)
{
  mcu_event();

  InterMcuEvent(handle_incoming_frame);

  ekf_event(); 
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
