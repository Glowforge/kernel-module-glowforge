/**
 * io.c
 *
 * I/O port and PWM configuration.
 *
 * Copyright (C) 2015-2021 Glowforge, Inc. <opensource@glowforge.com>
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
#include "io.h"

#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

void io_release_gpios(int *gpios, size_t ngpios)
{
  int i;
  for (i = 0; i < ngpios; i++) {
    int gpio = gpios[i];
    if (gpio >= 0) {
      gpio_free(gpio);
    }
  }
}


int io_init_gpios(struct device_node *of_node, const struct pin_config *pin_configs, int *gpios, size_t ngpios)
{
  int err = 0;
  int i;

  /* Read the pin mapping from the device tree node */
  for (i = 0; i < ngpios; i++) {
    const struct pin_config *pin = &pin_configs[i];
    int gpio = of_get_named_gpio(of_node, pin->name, 0);
    if (unlikely(gpio < 0)) {
      pr_err("no definition for GPIO pin \"%s\": %d\n", pin->name, gpio);
      err = gpio;
    }
    gpios[i] = gpio;
  }
  if (unlikely(err < 0)) {
    goto failed_devtree_lookup;
  }

  /* Request each gpio */
  for (i = 0; i < ngpios; i++) {
    const struct pin_config *pin = &pin_configs[i];
    int ret = gpio_request_one(gpios[i], pin->flags, pin->name);
    if (unlikely(ret < 0)) {
      pr_err("unable to request gpio%d (\"%s\"): %d\n", gpios[i], pin->name, ret);
      err = ret;
      gpios[i] = -1; /* so it doesn't get passed to gpio_free() */
    }
  }

  if (err == 0) {
    return 0;
  } else {
    io_release_gpios(gpios, ngpios);
  }

failed_devtree_lookup:
  return err;
}


u32 io_base_address(int *gpios, size_t ngpios, pin_set pin_bits)
{
  int i;
  u64 mask = 1;
  u32 min_addr = 0xFFFFFFFF;
  for (i = 0; i < ngpios; i++, mask <<= 1) {
    if (pin_bits & mask) {
      u32 addr = GPIO_BASE_ADDRESS(GPIO_BANK_FROM_PIN_NUMBER(gpios[i]));
      if (addr < min_addr) {
        min_addr = addr;
      }
    }
  }
  return min_addr;
}


void io_change_pins(int *gpios, size_t ngpios, const struct pin_change_set *change_set)
{
  int i;
  const struct pin_change *changes = change_set->changes;
  for (i = 0; i < change_set->len; i++) {
    pin_id pid = changes[i].pid;
    if (likely(pid >= 0 && pid <= ngpios)) {
      u8 value = changes[i].value;
      if (likely(value != HI_Z)) {
        gpio_direction_output(gpios[pid], value);
      } else {
        gpio_direction_input(gpios[pid]);
      }
    } else {
      pr_err("Pin ID %d out of range\n", pid);
    }
  }
}


void io_set_and_release_gpios(int *gpios, size_t ngpios, const struct pin_change_set *change_set)
{
  io_change_pins(gpios, ngpios, change_set);
  io_release_gpios(gpios, ngpios);
}


void io_release_pwms(struct pwm_channel *pwm_channels, size_t npwms)
{
  int i;
  for (i = 0; i < npwms; i++) {
    struct pwm_device *pwmdev = pwm_channels[i].pwmdev;
    if (likely(pwmdev)) {
      pwm_disable(pwmdev);
      pwm_free(pwmdev);
    }
  }
}


int io_init_pwms(struct device_node *of_node, const struct pwm_channel_config *pwm_configs, struct pwm_channel *pwm_channels, size_t npwms)
{
  int i;
  int ret;

  memset(pwm_channels, 0, npwms*sizeof(struct pwm_channel));
  for (i = 0; i < npwms; i++) {
    const struct pwm_channel_config *config = &pwm_configs[i];
    const char *name = config->name;
    struct pwm_channel *ch = &pwm_channels[i];
    struct pwm_device *pwmdev;
    int channel_num;

    if (unlikely(config->period == 0)) {
      pr_err("period must be nonzero for PWM channel \"%s\"\n", name);
      ret = -1;
      goto fail;
    }
    ch->period = config->period;

    if (unlikely(of_property_read_u32(of_node, name, &channel_num) != 0)) {
      pr_err("no definition found for PWM channel \"%s\"\n", name);
      ret = -1;
      goto fail;
    }

    pwmdev = pwm_request(channel_num, config->name);
    if (unlikely(IS_ERR(pwmdev))) {
      pr_err("failed to request PWM channel %d for \"%s\": %ld\n", channel_num, name, PTR_ERR(pwmdev));
      ch->pwmdev = NULL;
      ret = -1;
      goto fail;
    } else {
      ch->pwmdev = pwmdev;
    }
  }

  /*
   * Enable and initialize all channels with their designated frequencies,
   * but set their duty cycles to 0.
   * http://lists.infradead.org/pipermail/linux-arm-kernel/2012-August/115674.html
   * suggests that pwm_enable() must be called before pwm_config().
   */
  for (i = 0; i < npwms; i++) {
    struct pwm_channel *ch = &pwm_channels[i];
    if (!ch->pwmdev) { continue; }
    if ((ret = pwm_enable(ch->pwmdev))) {
      pr_err("unable to enable PWM channel %d\n", i);
      goto fail;
    }
    if ((ret = pwm_config(ch->pwmdev, 0, ch->period))) {
      pr_err("unable to set period of PWM channel %d\n", i);
      goto fail;
    }
  }

  return 0;

fail:
  io_release_pwms(pwm_channels, npwms);
  return ret;
}


void io_pwm_set_duty_cycle(struct pwm_channel *pwm_channel, u16 duty)
{
  /* Directly mapping the interval [0,65535] to the closed interval [0%,100%] */
  /* requires division by 65535 (and 64-bit division would be needed to avoid */
  /* overflow.) */
  /* For simplification, map 0 to 0%, and [1,65535] to [0.003%,100%]. */
  struct pwm_device *pwmdev = pwm_channel->pwmdev;
  if (pwmdev) {
    u32 period_ns = pwm_channel->period;
    u32 duty_ns = (duty) ? (period_ns*(duty+1)) >> 16 : 0;
    pwm_channel->duty_fraction = duty;
    pwm_config(pwmdev, duty_ns, period_ns);
  }
}


u16 io_pwm_get_duty_cycle(struct pwm_channel *pwm_channel)
{
  return pwm_channel->duty_fraction;
}


u32 io_pwm_sample_register_address(struct pwm_channel *pwm_channel)
{
  /* Get base register address */
  struct pwm_device *pwmdev = pwm_channel->pwmdev;
  if (pwmdev) {
    struct device *dev = pwmdev->chip->dev;
    struct platform_device *pdev = container_of(dev, struct platform_device, dev);
    struct resource *res = pdev->resource;
    /* PWMSAR is 12 bytes past the base address */
    return res->start + 0xc;
  } else {
    return 0;
  }
}
