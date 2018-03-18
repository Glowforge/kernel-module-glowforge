/**
 * pic_leds.c
 *
 * Linux LED class interface.
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

#include "pic_private.h"

#include <linux/workqueue.h>

/* LED PWMs have 10 bits of resolution */
#define PIC_LED_MAX_BRIGHTNESS  1023

int ledtrig_smooth_init(void);
int ledtrig_smooth_remove(void);

/**
 * LED set commands are offloaded to a work queue; this struct encapsulates
 * the data necessary to perform such work.
 */
struct pic_led_work {
  struct work_struct work;
  struct pic *self;
  enum pic_register reg;
  pic_value value;
};


#define CONFIGURE_LED_CLASSDEV(self, led) do { \
  self->led.name = #led; \
  self->led.default_trigger = "smooth"; \
  self->led.brightness_get = led##_get; \
  self->led.brightness_set = led##_set; \
  self->led.max_brightness = PIC_LED_MAX_BRIGHTNESS; } while (0)

#define REGISTER_LED_CLASSDEV(self, led) do { \
  ret = led_classdev_register(self->dev, &self->led); \
  if (ret) { \
    dev_err(self->dev, "failed to register " #led); \
    goto failed_register_led; \
  } } while (0)

#define UNREGISTER_LED_CLASSDEV(self, led) \
  if (!IS_ERR_OR_NULL(self->led.dev)) { \
    led_classdev_unregister(&self->led); \
  }

#define DEFINE_LED_CALLBACKS(led, reg) \
  static void led##_set(struct led_classdev *led_cdev, enum led_brightness brightness) { \
    struct pic *self = container_of(led_cdev, struct pic, led); \
    struct spi_device *spi = to_spi_device(self->dev); \
    pic_led_set(spi, reg, brightness); } \
  static enum led_brightness led##_get(struct led_classdev *led_cdev) { \
    struct pic *self = container_of(led_cdev, struct pic, led); \
    struct spi_device *spi = to_spi_device(self->dev); \
    return pic_led_get(spi, reg); }


static void pic_led_set_work(struct work_struct *work)
{
  struct pic_led_work *led_work = container_of(work, struct pic_led_work, work);
  pic_write_one_register(to_spi_device(led_work->self->dev), led_work->reg, led_work->value);
  kfree(led_work);
}


static void pic_led_set(struct spi_device *spi, enum pic_register reg, enum led_brightness brightness)
{
  struct pic *self = spi_get_drvdata(spi);
  struct pic_led_work *led_work;

  if (brightness > PIC_LED_MAX_BRIGHTNESS) { brightness = PIC_LED_MAX_BRIGHTNESS; }

  led_work = kzalloc(sizeof(*led_work), GFP_ATOMIC);
  if (!led_work) { return; }
  INIT_WORK(&led_work->work, pic_led_set_work);
  led_work->self = self;
  led_work->reg = reg;
  led_work->value = brightness;
  schedule_work(&led_work->work);
}


static enum led_brightness pic_led_get(struct spi_device *spi, enum pic_register reg)
{
  return pic_read_one_register(spi, reg);
}


DEFINE_LED_CALLBACKS(lid_led, PIC_REG_LID_LED)
DEFINE_LED_CALLBACKS(button_led_1, PIC_REG_BUTTON_LED1)
DEFINE_LED_CALLBACKS(button_led_2, PIC_REG_BUTTON_LED2)
DEFINE_LED_CALLBACKS(button_led_3, PIC_REG_BUTTON_LED3)


int pic_register_leds(struct spi_device *spi)
{
  struct pic *self = spi_get_drvdata(spi);
  int ret;

  ret = ledtrig_smooth_init();
  if (ret) {
    return ret;
  }

  CONFIGURE_LED_CLASSDEV(self, lid_led);
  CONFIGURE_LED_CLASSDEV(self, button_led_1);
  CONFIGURE_LED_CLASSDEV(self, button_led_2);
  CONFIGURE_LED_CLASSDEV(self, button_led_3);
  REGISTER_LED_CLASSDEV(self, lid_led);
  REGISTER_LED_CLASSDEV(self, button_led_1);
  REGISTER_LED_CLASSDEV(self, button_led_2);
  REGISTER_LED_CLASSDEV(self, button_led_3);
  return 0;

failed_register_led:
  pic_unregister_leds(spi);
  return ret;
}


void pic_unregister_leds(struct spi_device *spi)
{
  struct pic *self = spi_get_drvdata(spi);
  ledtrig_smooth_remove();
  UNREGISTER_LED_CLASSDEV(self, lid_led);
  UNREGISTER_LED_CLASSDEV(self, button_led_1);
  UNREGISTER_LED_CLASSDEV(self, button_led_2);
  UNREGISTER_LED_CLASSDEV(self, button_led_3);
  flush_scheduled_work();
}
