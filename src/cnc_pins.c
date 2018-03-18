/**
 * cnc_pins.c
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
#include "cnc_pins.h"

/**
 * Names and flags for each GPIO pin.
 */
const struct pin_config pin_configs[NUM_GPIO_PINS] = {
  [PIN_X_ENABLE]            = {"x-enable-gpio",      GPIOF_OUT_INIT_HIGH}, /* active low */
  [PIN_X_STEP]              = {"x-step-gpio",        GPIOF_OUT_INIT_LOW},
  [PIN_X_DIR]               = {"x-dir-gpio",         GPIOF_OUT_INIT_LOW},
  [PIN_X_HOME]              = {"x-home-gpio",        GPIOF_IN},
  [PIN_X_FAULT]             = {"x-fault-gpio",       GPIOF_IN},
  [PIN_X_MODE0]             = {"x-mode0-gpio",       GPIOF_OUT_INIT_LOW},
  [PIN_X_MODE1]             = {"x-mode1-gpio",       GPIOF_OUT_INIT_LOW},
  [PIN_X_MODE2]             = {"x-mode2-gpio",       GPIOF_OUT_INIT_LOW},
  [PIN_X_DECAY]             = {"x-decay-gpio",       GPIOF_IN}, /* HiZ = mixed decay */
  [PIN_Y1_ENABLE]           = {"y1-enable-gpio",     GPIOF_OUT_INIT_HIGH}, /* active low */
  [PIN_Y1_STEP]             = {"y1-step-gpio",       GPIOF_OUT_INIT_LOW},
  [PIN_Y1_DIR]              = {"y1-dir-gpio",        GPIOF_OUT_INIT_LOW},
  [PIN_Y1_HOME]             = {"y1-home-gpio",       GPIOF_IN},
  [PIN_Y1_FAULT]            = {"y1-fault-gpio",      GPIOF_IN},
  [PIN_Y2_ENABLE]           = {"y2-enable-gpio",     GPIOF_OUT_INIT_HIGH}, /* active low */
  [PIN_Y2_STEP]             = {"y2-step-gpio",       GPIOF_OUT_INIT_LOW},
  [PIN_Y2_DIR]              = {"y2-dir-gpio",        GPIOF_OUT_INIT_LOW},
  [PIN_Y2_HOME]             = {"y2-home-gpio",       GPIOF_IN},
  [PIN_Y2_FAULT]            = {"y2-fault-gpio",      GPIOF_IN},
  [PIN_Y_MODE0]             = {"y-mode0-gpio",       GPIOF_OUT_INIT_LOW},
  [PIN_Y_MODE1]             = {"y-mode1-gpio",       GPIOF_OUT_INIT_LOW},
  [PIN_Y_MODE2]             = {"y-mode2-gpio",       GPIOF_OUT_INIT_LOW},
  [PIN_Y_DECAY]             = {"y-decay-gpio",       GPIOF_IN}, /* HiZ = mixed decay */
  [PIN_STEP_RESET]          = {"reset-gpio",         GPIOF_OUT_INIT_LOW}, /* high = stepper driver operation, low = reset */
  [PIN_STEP_SLEEP]          = {"sleep-gpio",         GPIOF_OUT_INIT_LOW}, /* high = chip enabled, low = sleep mode */
  [PIN_LASER_ON]            = {"laser-enable-gpio",  GPIOF_IN}, /* kept HiZ until explicitly enabled */
  [PIN_LASER_ON_HEAD]       = {"laser-on-head-gpio", GPIOF_OUT_INIT_LOW}, /* informative signal for the head */
  [PIN_CHARGE_PUMP]         = {"charge-pump-gpio",   GPIOF_OUT_INIT_LOW},
  [PIN_Z_ENABLE]            = {"z-enable-gpio",      GPIOF_OUT_INIT_LOW}, /* active high */
  [PIN_Z_STEP]              = {"z-step-gpio",        GPIOF_OUT_INIT_LOW},
  [PIN_Z_DIR]               = {"z-dir-gpio",         GPIOF_OUT_INIT_LOW},
  [PIN_LASER_LATCH_RESET]   = {"latch-reset-gpio",   GPIOF_OUT_INIT_HIGH},
  [PIN_BEAM_LATCH_RESET]    = {"beam-reset-gpio",    GPIOF_OUT_INIT_LOW},
};


const pin_set cnc_sdma_pin_set =
  (1ULL << PIN_X_STEP)  |
  (1ULL << PIN_X_DIR)   |
  (1ULL << PIN_Y1_STEP) |
  (1ULL << PIN_Y1_DIR)  |
  (1ULL << PIN_Y2_STEP) |
  (1ULL << PIN_Y2_DIR)  |
  (1ULL << PIN_Z_STEP)  |
  (1ULL << PIN_Z_DIR)   |
  (1ULL << PIN_LASER_ON);


/** Pin changes to apply after initialization, or when reenabling the driver. */
DEFINE_PIN_CHANGE_SET(cnc_startup_pin_changes,
  {PIN_STEP_RESET, 1},
  {PIN_STEP_SLEEP, 1},
  {PIN_X_ENABLE, 0}, /* active low */
  {PIN_Y1_ENABLE, 0}, /* active low */
  {PIN_Y2_ENABLE, 0}, /* active low */
  {PIN_Z_ENABLE, 1}, /* active high */
  {PIN_CHARGE_PUMP, 0},
  {PIN_X_STEP, 0},
  {PIN_X_DIR, 0},
  {PIN_Y1_STEP, 0},
  {PIN_Y1_DIR, 0},
  {PIN_Y2_STEP, 0},
  {PIN_Y2_DIR, 0},
  {PIN_Z_STEP, 0},
  {PIN_Z_DIR, 0},
);


/**
 * Pin changes to apply when stopping a cut normally.
 * Brings all drive lines low but keeps the steppers powered up.
 */
DEFINE_PIN_CHANGE_SET(cnc_stop_pin_changes,
  {PIN_LASER_ON, HI_Z},
  {PIN_CHARGE_PUMP, 0},
  {PIN_X_STEP, 0},
  {PIN_X_DIR, 0},
  {PIN_Y1_STEP, 0},
  {PIN_Y1_DIR, 0},
  {PIN_Y2_STEP, 0},
  {PIN_Y2_DIR, 0},
  {PIN_Z_STEP, 0},
  {PIN_Z_DIR, 0},
);


/**
 * Pin changes to apply when the driver is disabled or the module is unloaded.
 * Brings all drive lines low and powers off the steppers.
 */
DEFINE_PIN_CHANGE_SET(cnc_shutdown_pin_changes,
  {PIN_STEP_RESET, 0},
  {PIN_STEP_SLEEP, 0},
  {PIN_LASER_LATCH_RESET, 1},
  {PIN_X_ENABLE, 1}, /* active low */
  {PIN_Y1_ENABLE, 1}, /* active low */
  {PIN_Y2_ENABLE, 1}, /* active low */
  {PIN_Z_ENABLE, 0}, /* active high */
  {PIN_LASER_ON, HI_Z},
  {PIN_CHARGE_PUMP, 0},
  {PIN_X_STEP, 0},
  {PIN_X_DIR, 0},
  {PIN_Y1_STEP, 0},
  {PIN_Y1_DIR, 0},
  {PIN_Y2_STEP, 0},
  {PIN_Y2_DIR, 0},
  {PIN_Z_STEP, 0},
  {PIN_Z_DIR, 0},
);
