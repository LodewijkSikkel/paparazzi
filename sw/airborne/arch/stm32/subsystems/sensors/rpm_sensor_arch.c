/*
 * Copyright (C) Bart Slinger
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/rpm_sensor/rpm_sensor.c"
 * @author Bart Slinger
 * Measure the ppm signal of the RPM sensor
 */

#include "subsystems/sensors/rpm_sensor.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "mcu_periph/gpio.h"

// for timer_get_frequency
#include "mcu_arch.h"

// Telemetry to test values
#include "subsystems/datalink/telemetry.h"
#include "led.h"

#define ONE_MHZ_CLK 1000000
#ifdef NVIC_TIM_IRQ_PRIO
#define RPM_PPM_IRQ_PRIO  NVIC_TIM_IRQ_PRIO
#else
#define RPM_PPM_IRQ_PRIO 2
#endif

#define RPM_RCC_TIM_PPM         RCC_TIM5
#define RPM_PPM_TIMER           TIM5
#define RPM_PPM_CHANNEL         TIM_IC2
#define RPM_PPM_STATUS_REG      TIM5_SR
#define RPM_PPM_GPIO_PORT       GPIOA
#define RPM_PPM_GPIO_PIN        GPIO1
#define RPM_PPM_GPIO_AF         0
#define RPM_PPM_GPIO_REMAP      AFIO_MAPR_TIM2_REMAP_FULL_REMAP // remap GPIO pin to use TIM2C3
#define RPM_PPM_TIMER_INPUT     TIM_IC_IN_TI2 // channel 2 is used

#define RPM_PPM_IRQ             NVIC_TIM5_IRQ // nested vector TIM2 interrupt request
#define RPM_PPM_CC_IE           TIM_DIER_CC2IE // capture/compare 1 interrupt enable 
#define RPM_PPM_CC_IF           TIM_SR_CC2IF // capture/compare 1 interrupt flag

static uint8_t rpm_sensor_arch_overflow_cnt;

void rpm_sensor_arch_init(void)
{
  /* timer peripheral clock enable */
  rcc_periph_clock_enable(RPM_RCC_TIM_PPM);
  rcc_periph_clock_enable(RCC_AFIO);

  /* GPIO configuration as input capture for timer
   * Using GPIO pin on Lisa/M v2.0 (=PA1).
   * Requires remapping to get timer 5 channel 2.
   */
  gpio_enable_clock(RPM_PPM_GPIO_PORT);
  gpio_primary_remap(FALSE, (0x3<<8));
  gpio_set_mode(RPM_PPM_GPIO_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, RPM_PPM_GPIO_PIN);

  /* Time Base configuration */
  timer_reset(RPM_PPM_TIMER);
  timer_set_mode(RPM_PPM_TIMER, TIM_CR1_CKD_CK_INT, // clock division ratio
                 TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_period(RPM_PPM_TIMER, 0xFFFF);
  timer_set_prescaler(RPM_PPM_TIMER, 256);

  /* TIM configuration: Input Capture mode 
   * The Rising edge is used as active edge 
   */
  timer_ic_set_polarity(RPM_PPM_TIMER, RPM_PPM_CHANNEL, TIM_IC_RISING);
  timer_ic_set_input(RPM_PPM_TIMER, RPM_PPM_CHANNEL, RPM_PPM_TIMER_INPUT);
  timer_ic_set_prescaler(RPM_PPM_TIMER, RPM_PPM_CHANNEL, TIM_IC_PSC_OFF);
  timer_ic_set_filter(RPM_PPM_TIMER, RPM_PPM_CHANNEL, TIM_IC_OFF);

  /* Enable timer Interrupt(s). */
  nvic_set_priority(RPM_PPM_IRQ, RPM_PPM_IRQ_PRIO);
  nvic_enable_irq(RPM_PPM_IRQ);

  /* Enable the Capture/Compare and Update interrupt requests. */
  timer_enable_irq(RPM_PPM_TIMER, (RPM_PPM_CC_IE | TIM_DIER_UIE));

  /* Enable capture channel. */
  timer_ic_enable(RPM_PPM_TIMER, RPM_PPM_CHANNEL);

  /* TIM enable counter */
  timer_enable_counter(RPM_PPM_TIMER);
}

void tim5_isr(void)
{
  if ((RPM_PPM_STATUS_REG & RPM_PPM_CC_IF) != 0) { /* Interrupt from rising edge */
    timer_clear_flag(RPM_PPM_TIMER, RPM_PPM_CC_IF);

    uint16_t now = timer_get_counter(RPM_PPM_TIMER);
    rpm_sensor_process_pulse(now, rpm_sensor_arch_overflow_cnt);
    rpm_sensor_arch_overflow_cnt = 0;

  } else if ((TIM5_SR & TIM_SR_UIF) != 0) { /* Interrupt from overflow */
    timer_clear_flag(RPM_PPM_TIMER, TIM_SR_UIF);

    if (rpm_sensor_arch_overflow_cnt < 2) {
      rpm_sensor_arch_overflow_cnt++;
    }

  }
}