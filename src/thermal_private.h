/**
 * thermal_private.h
 *
 * Private header for the thermal interface.
 *
 * Copyright (C) 2015-2018 Glowforge, Inc. <opensource@glowforge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef KERNEL_SRC_THERMAL_PRIVATE_H_
#define KERNEL_SRC_THERMAL_PRIVATE_H_

#include "io.h"
#include "notifiers.h"
#include <linux/platform_device.h>
#include <linux/hrtimer.h>

enum {
  PIN_WATER_PUMP,
  PIN_WATER_HEATER,
  PIN_TEC,
  PIN_INTAKE1_TACH,
  PIN_INTAKE2_TACH,
  PIN_EXHAUST_TACH,
  THERMAL_NUM_GPIO_PINS
};

enum {
  PWM_INTAKE,
  PWM_EXHAUST,
  THERMAL_NUM_PWM_CHANNELS
};

enum {
  TACH_INTAKE1,
  TACH_INTAKE2,
  TACH_EXHAUST,
  THERMAL_NUM_TACH_SIGNALS
};


/**
 * Frequencies below MINIMUM_TACH_FREQUENCY_HZ will read as 0
 */
#define MINIMUM_TACH_FREQUENCY_HZ 10
#define MAXIMUM_TACH_PERIOD_NS (1000000000UL/MINIMUM_TACH_FREQUENCY_HZ)

struct tach_channel {
  int irq_num;
  spinlock_t lock;
  /** these two times are shared between IRQ handler and tach_show() */
  ktime_t last_period;  /** period as of last IRQ */
  ktime_t last_edge;  /** last edge IRQ time */
};

struct thermal {
  /** Device that owns this data */
  struct device *dev;
  /** GPIO pins. */
  int gpios[THERMAL_NUM_GPIO_PINS];
  /** PWM channels. */
  struct pwm_channel pwms[THERMAL_NUM_PWM_CHANNELS];
  /** Heater control uses a low frequency PWM controlled by software. */
  struct hrtimer heater_pwm_timer;
  /** Heater PWM duty cycle; 0=0%, 65535=100% */
  u16 heater_duty_fraction;
  /** Tach data. */
  struct tach_channel tachs[THERMAL_NUM_TACH_SIGNALS];
  /** Notifiers */
  struct notifier_block dms_notifier;
};

#endif
