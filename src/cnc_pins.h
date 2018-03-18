/**
 * cnc_pins.h
 *
 * Stepper driver GPIO pin mapping.
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

#ifndef KERNEL_SRC_CNC_PINS_H_
#define KERNEL_SRC_CNC_PINS_H_

#include "io.h"

enum {
  PIN_X_ENABLE,
  PIN_X_STEP,
  PIN_X_DIR,
  PIN_X_HOME,
  PIN_X_FAULT,
  PIN_X_MODE0,
  PIN_X_MODE1,
  PIN_X_MODE2,
  PIN_X_DECAY,
  PIN_Y1_ENABLE,
  PIN_Y1_STEP,
  PIN_Y1_DIR,
  PIN_Y1_HOME,
  PIN_Y1_FAULT,
  PIN_Y2_ENABLE,
  PIN_Y2_STEP,
  PIN_Y2_DIR,
  PIN_Y2_HOME,
  PIN_Y2_FAULT,
  PIN_Y_MODE0,
  PIN_Y_MODE1,
  PIN_Y_MODE2,
  PIN_Y_DECAY,
  PIN_Z_ENABLE,
  PIN_Z_STEP,
  PIN_Z_DIR,
  PIN_STEP_RESET,
  PIN_STEP_SLEEP,
  PIN_LASER_ON,
  PIN_LASER_ON_HEAD,
  PIN_CHARGE_PUMP,
  PIN_LASER_LATCH_RESET,
  PIN_BEAM_LATCH_RESET,
  /* ---------- */
  NUM_GPIO_PINS
};



/**
 * Pin changes to apply at startup after successful initialization.
 */
extern const struct pin_change_set *cnc_startup_pin_changes;

/**
 * Pin changes to apply when stopping a cut normally.
 * Brings all drive lines low but keeps the steppers powered up.
 */
extern const struct pin_change_set *cnc_stop_pin_changes;

/**
 * Pin changes to apply when there is an emergency stop, or when unloading
 * the driver.
 * Brings all drive lines low and powers off the steppers.
 */
extern const struct pin_change_set *cnc_shutdown_pin_changes;

/**
 * A set indicating which pins are controlled by the SDMA script.
 */
extern const pin_set cnc_sdma_pin_set;

/**
 * Array of pin configurations.
 */
extern const struct pin_config pin_configs[NUM_GPIO_PINS];

#endif
