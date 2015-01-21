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
  DOWNLINK_SEND_ESC32(DefaultChannel, DefaultDevice, &actuators_can_data.next_node_slot, 4, (uint8_t *)&esc32_state);
}

