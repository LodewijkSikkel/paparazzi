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
#include "led.h"
#include "subsystems/intermcu/intermcu_standalone.h"

// #ifdef INTERMCU_AP
// #define DATALINK_C // included such that datalink_nb_msgs is defined in the datalink.h header file
// #include "subsystems/datalink/downlink.h"
// #endif

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);

  intermcu_init(); // initialize the inter-mcu connection

// #ifdef INTERMCU_AP
//   downlink_init(); // initialize the telemetry downlink
// #endif
}

static inline void main_periodic_task(void)
{ 
  RunOnceEvery(10, {
    LED_TOGGLE(2);
  });

  intermcu_periodic(); // check the connection 
}

static inline void handle_incoming_frame(void) 
{
}

static inline void main_event_task(void)
{
  mcu_event();

  InterMcuEvent(handle_incoming_frame); 
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
