#include "esc32_telemetry.h"
#include "subsystems/actuators/actuators_can.h"
#include "subsystems/actuators/actuators_esc32.h"

#include "mcu_periph/uart.h"

#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include BOARD_CONFIG

void esc32_telemetry_init(void)
{
}

void esc32_telemetry_periodic(void)
{
  uint16_t vin[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++)
    vin[i] = motors_data.can_status[i].vin;

  DOWNLINK_SEND_ESC32(DefaultChannel, DefaultDevice, &can_data.next_node_slot, &can_data.timeouts, 4, vin);
}

