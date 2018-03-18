/**
 * io.h
 *
 * I/O port and PWM configuration.
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

#ifndef KERNEL_SRC_IO_H_
#define KERNEL_SRC_IO_H_

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/pwm.h>

#pragma mark - Data types/macros

/**
 * Macros for conversion between Linux GPIO numbers, bank/pin pairs,
 * and bank register addresses. (i.MX6)
 */
#define GPIO_PIN_NUMBER(bank,pin)       (((bank)-1)*32 + pin)
#define GPIO_BANK_FROM_PIN_NUMBER(gpio) (((gpio)/32)+1)
#define GPIO_BASE_ADDRESS(bank)         (0x209C000 + ((bank)-1)*0x4000)
#define PIN_FROM_GPIO(gpio)             ((gpio) & 31)

/**
 * Data type that can represent a set of GPIO pin indices.
 */
typedef u64 pin_set;

/**
 * Struct representing a pin configuration;
 * its name and flags (direction and initial value).
 * The name is used to look up the pin's bank and pin number in the device tree.
 */
struct pin_config {
  const char *name;
  const u32 flags;  /* GPIOF_* flags from linux/gpio.h */
};


/**
 * Struct representing a change to be applied to a GPIO pin.
 */
typedef int pin_id;
struct pin_change {
  pin_id pid;  /* pin enum */
  u8 value; /* 1 for high, 0 for low, 0xFF for HiZ */
};
#define HI_Z 0xFF

/**
 * Struct representing a set of pin level changes.
 */
struct pin_change_set {
  const struct pin_change *changes;
  size_t len;
};


/**
 * Convenience macro for defining a pin change set.
 */
#define DEFINE_PIN_CHANGE_SET(setname, ...) \
  const struct pin_change _##setname##_arr[] = { __VA_ARGS__ }; \
  const struct pin_change_set _##setname = { _##setname##_arr, sizeof(_##setname##_arr)/sizeof(_##setname##_arr[0]) }; \
  const struct pin_change_set *setname = &_##setname;


/**
 * Struct representing a PWM channel configuration: its name and period.
 * The name is used to look up the channel number in the device tree.
 */
struct pwm_channel_config {
  const char *name;
  const u32 period; /* in nanoseconds */
};


struct pwm_channel {
  struct pwm_device *pwmdev;
  u32 period; /* in nanoseconds */
  u16 duty_fraction; /* 0=0%, 65535=100% */
};



#pragma mark - Functions

/**
 * Allocates a set of GPIOs from a device tree node.
 *
 * @param of_node     Pointer to the device tree node that specifies GPIO mapping
 * @param pin_configs Array of pin configs to search the device tree node for
 * @param gpios       Array of GPIO numbers to populate
 * @param ngpios      Number of GPIOs in the preceding arrays
 * @return            0 on success, error code on failure
 */
int io_init_gpios(struct device_node *of_node, const struct pin_config *pin_configs, int *gpios, size_t ngpios);

/**
 * Returns the address of the lowest GPIO bank for the given pin indexes.
 *
 * @param gpios     Array of GPIOs
 * @param ngpios    Size of the gpios array
 * @param pin_bits  Bit mask; a 1 bit in the nth place includes pin with ID
 * @return          The lowest bank address of all pins in the specified set
 *
 * @note 1 bits in positions >= NUM_GPIO_PINS are ignored.
 * @note Will not work for GPIO pins whose index is greater than the maximum
 *       bit position in the pin_set type.
 */
u32 io_base_address(int *gpios, size_t ngpios, pin_set pin_bits);

/**
 * Applies the given set of GPIO pin changes.
 *
 * @param gpios       Array of GPIOs
 * @param ngpios      Size of the gpios array
 * @param change_set  Pin change set to apply.
 */
void io_change_pins(int *gpios, size_t ngpios, const struct pin_change_set *change_set);

/**
 * Releases a set of GPIOs.
 *
 * @param gpios   Array of GPIOs to release
 * @param ngpios  Size of the gpios array
 */
void io_release_gpios(int *gpios, size_t ngpios);

/**
 * Applies a set of pin changes, then releases a set of GPIOs.
 * Useful in a cleanup function to restore pins to a safe state before
 * releasing them.
 *
 * @param gpios       Array of GPIOs to release
 * @param ngpios      Size of the gpios array
 * @param change_set  Pin changes to apply before releasing the GPIOs
 */
void io_set_and_release_gpios(int *gpios, size_t ngpios, const struct pin_change_set *change_set);

/**
 * Initializes a set of PWM channels.
 *
 * @param of_node       Pointer to the device tree node that specifies channel mapping
 * @param pwm_configs   Array of configurations to use for each channel
 * @param pwm_channels  Channels to populate and initialize
 * @param npwms         Number of PWM channels in the preceding arrays
 * @return              0 on success, error code on failure
 */
int io_init_pwms(struct device_node *of_node, const struct pwm_channel_config *pwm_configs, struct pwm_channel *pwm_channels, size_t npwms);

/**
 * Sets the duty cycle (percentage) of the given PWM channel.
 *
 * @param pwm_channel Channel to update
 * @param duty        Desired duty cycle (0=0%, 65535=100%)
 */
void io_pwm_set_duty_cycle(struct pwm_channel *pwm_channel, u16 duty);

/**
 * Retrieves the current duty cycle setting of the given PWM channel.
 *
 * @param pwm_channel Channel
 * @return            Duty cycle (0=0%, 65535=100%)
 */
u16 io_pwm_get_duty_cycle(struct pwm_channel *pwm_channel);

/**
 * Releases a set of PWM channels.
 * All are set to a duty cycle of 0 before being released.
 *
 * @param gpios   Array of GPIOs to release
 * @param ngpios  Size of the gpios array
 */
void io_release_pwms(struct pwm_channel *pwm_channels, size_t npwms);

/**
 * Returns the physical address of the PWM channel's sample (duty cycle)
 * register. (i.MX6-specific)
 */
u32 io_pwm_sample_register_address(struct pwm_channel *pwm_channel);

/**
 * Converts a frequency in hertz to a period in nanoseconds.
 */
#define HZ_TO_PERIOD_NS(hz) (1000000000/(hz))

/**
 * Converts a resolution in bits to a period in nanoseconds.
 * With the given period, a PWM sample of 0 corresponds to 0% duty cycle,
 * and a sample of (1<<bits)-1 corresponds to 100% duty cycle.
 */
#define PWM_ROOT_FREQ_MHZ       66  /* i.MX6-specific */
#define BITS_TO_PERIOD_NS(bits) DIV_ROUND_UP(((1<<(bits))-1)*1000, PWM_ROOT_FREQ_MHZ)

#endif
