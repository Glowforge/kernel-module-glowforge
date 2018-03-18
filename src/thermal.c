/**
 * thermal.c
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
#include "thermal_private.h"
#include "device_attr.h"
#include "io.h"
#include "uapi/glowforge.h"
#include <linux/interrupt.h>

/** Interval between fan speed measurements */
#define TACH_SWITCH_TIME_NS   2.5e8

/** PWM period for heater control */
#define HEATER_PWM_PERIOD_NS  1e7

/** Heater maximum duty cycle. Since heater_duty_fraction is a u16, this value is 65535. */
#define HEATER_DUTY_CYCLE_BITS  (FIELD_SIZEOF(struct thermal, heater_duty_fraction)*8)
#define HEATER_DUTY_MAX         ((1 << HEATER_DUTY_CYCLE_BITS)-1)

static const ktime_t heater_pwm_period = { .tv64 = HEATER_PWM_PERIOD_NS };
static const ktime_t ktime_zero = {0};
static void thermal_set_heater_duty_fraction(struct thermal *self, u16 duty_fraction);



extern struct kobject *glowforge_kobj;

/** Module parameters */
extern int thermal_enabled;

#pragma mark - GPIO/PWM definitions

static const struct pin_config thermal_pin_configs[THERMAL_NUM_GPIO_PINS] = {
  [PIN_WATER_PUMP]      = {"water-pump-gpio",      GPIOF_OUT_INIT_HIGH}, /* pump should be on at start */
  [PIN_WATER_HEATER]    = {"water-heater-gpio",    GPIOF_OUT_INIT_LOW},
  [PIN_TEC]             = {"tec-gpio",             GPIOF_OUT_INIT_LOW},
  [PIN_INTAKE1_TACH]    = {"intake1-tach-gpio",    GPIOF_IN},
  [PIN_INTAKE2_TACH]    = {"intake2-tach-gpio",    GPIOF_IN},
  [PIN_EXHAUST_TACH]    = {"exhaust-tach-gpio",    GPIOF_IN},
};

static const struct pwm_channel_config thermal_pwm_configs[THERMAL_NUM_PWM_CHANNELS] = {
  [PWM_INTAKE]     = {"intake-pwm",     HZ_TO_PERIOD_NS(25000)},
  [PWM_EXHAUST]    = {"exhaust-pwm",    HZ_TO_PERIOD_NS(25000)},
};

static const pin_id tach_pin_ids[THERMAL_NUM_TACH_SIGNALS] = {
  [TACH_INTAKE1]    = PIN_INTAKE1_TACH,
  [TACH_INTAKE2]    = PIN_INTAKE2_TACH,
  [TACH_EXHAUST]    = PIN_EXHAUST_TACH,
};

/**
 * Pin changes to apply when the driver is unloaded.
 */
DEFINE_PIN_CHANGE_SET(thermal_shutdown_pin_changes,
  {PIN_WATER_PUMP, 0},
  {PIN_WATER_HEATER, 0},
  {PIN_TEC, 0},
);



#pragma mark - Deadman switch handler

/* Deadman switch trip notification */
static void thermal_make_safe(struct thermal *self)
{
  int i;
  io_change_pins(self->gpios, THERMAL_NUM_GPIO_PINS, thermal_shutdown_pin_changes);
  for (i = 0; i < THERMAL_NUM_PWM_CHANNELS; i++) {
    io_pwm_set_duty_cycle(&self->pwms[i], 0);
  }
  dev_err(self->dev, "making safe");
}


static int thermal_dms_handler(struct notifier_block *nb, unsigned long action, void *data)
{
  struct thermal *self = container_of(nb, struct thermal, dms_notifier);
  thermal_make_safe(self);
  return 0;
}


#pragma mark - Sysfs API

/** Must be called with tach spinlock held */
static inline u64 get_tach_value_ns(struct tach_channel *tach)
{
  ktime_t last_edge = ktime_sub(ktime_get(), tach->last_edge);
  u64 last_edge_ns = ktime_to_ns(last_edge);
  u64 last_period_ns = ktime_to_ns(tach->last_period);

  /* If the last edge or last recorded period was too long ago, we have stalled */
  return ((last_period_ns < MAXIMUM_TACH_PERIOD_NS && last_edge_ns < MAXIMUM_TACH_PERIOD_NS)
      ? last_period_ns : 0);
}

#define DEFINE_PWM_ATTR(name, pwm) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    struct thermal *self = dev_get_drvdata(dev); \
    return scnprintf(buf, PAGE_SIZE, "%hu\n", io_pwm_get_duty_cycle(&self->pwms[pwm])); } \
  static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
    struct thermal *self = dev_get_drvdata(dev); \
    unsigned long new_value; \
    ssize_t ret = kstrtoul(buf, 10, &new_value); \
    if (ret) { return ret; } \
    if (new_value > 65535) { return -EINVAL; } \
    io_pwm_set_duty_cycle(&self->pwms[pwm], new_value); \
    return count; } \
  static DEVICE_ATTR(name, S_IRUSR|S_IWUSR, name##_show, name##_store)

#define DEFINE_GPIO_ATTR(name, pin) \
  static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
    struct thermal *self = dev_get_drvdata(dev); char ch; \
    if (count < 1) { return -EINVAL; } \
    ch = *buf; if (ch != '0' && ch != '1') { return -EINVAL; } \
    gpio_set_value(self->gpios[pin], ch == '1'); \
    return count; } \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    struct thermal *self = dev_get_drvdata(dev); \
    return scnprintf(buf, PAGE_SIZE, "%d\n", (gpio_get_value(self->gpios[pin]) != 0)); } \
  static DEVICE_ATTR(name, S_IRUSR|S_IWUSR, name##_show, name##_store)

#define DEFINE_TACH_ATTR(name, tachnum) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    u64 reported_value_ns; \
    unsigned long flags; \
    struct thermal *self = dev_get_drvdata(dev); \
    struct tach_channel *tach = &self->tachs[tachnum]; \
    spin_lock_irqsave(&tach->lock, flags); \
    reported_value_ns = get_tach_value_ns(tach); \
    spin_unlock_irqrestore(&tach->lock, flags); \
    return scnprintf(buf, PAGE_SIZE, "%llu\n", reported_value_ns); } \
  static DEVICE_ATTR(name, S_IRUSR, name##_show, NULL)

static ssize_t heater_pwm_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct thermal *self = dev_get_drvdata(dev);
  return scnprintf(buf, PAGE_SIZE, "%hu\n", self->heater_duty_fraction);
}

static ssize_t heater_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
  struct thermal *self = dev_get_drvdata(dev);
  unsigned long new_value; \
  ssize_t ret = kstrtoul(buf, 10, &new_value);
  if (ret) { return ret; }
  if (new_value > HEATER_DUTY_MAX) { return -EINVAL; }
  thermal_set_heater_duty_fraction(self, new_value);
  return count;
}

DEFINE_PWM_ATTR(ATTR_INTAKE_PWM,     PWM_INTAKE);
DEFINE_PWM_ATTR(ATTR_EXHAUST_PWM,    PWM_EXHAUST);
static DEVICE_ATTR(heater_pwm, S_IRUSR|S_IWUSR, heater_pwm_show, heater_pwm_store);
DEFINE_GPIO_ATTR(ATTR_WATER_PUMP_ON, PIN_WATER_PUMP);
DEFINE_GPIO_ATTR(ATTR_TEC_ON,        PIN_TEC);
DEFINE_TACH_ATTR(ATTR_INTAKE1_TACH,  TACH_INTAKE1);
DEFINE_TACH_ATTR(ATTR_INTAKE2_TACH,  TACH_INTAKE2);
DEFINE_TACH_ATTR(ATTR_EXHAUST_TACH,  TACH_EXHAUST);

static struct attribute *thermal_attrs[] = {
  DEV_ATTR_PTR(ATTR_INTAKE_PWM),
  DEV_ATTR_PTR(ATTR_EXHAUST_PWM),
  DEV_ATTR_PTR(ATTR_HEATER_PWM),
  DEV_ATTR_PTR(ATTR_WATER_PUMP_ON),
  DEV_ATTR_PTR(ATTR_TEC_ON),
  DEV_ATTR_PTR(ATTR_INTAKE1_TACH),
  DEV_ATTR_PTR(ATTR_INTAKE2_TACH),
  DEV_ATTR_PTR(ATTR_EXHAUST_TACH),
  NULL
};

const struct attribute_group thermal_attr_group = {
  .attrs = thermal_attrs
};



#pragma mark - Interrupt handlers

/**
 * IRQ handler called on every rising edge of fan tach signals.
 */
static irqreturn_t tach_irq_handler(unsigned int irq, void *dev_id)
{
  struct tach_channel *tach = (struct tach_channel *)dev_id;
  ktime_t now = ktime_get();

  tach->last_period = ktime_sub(now, tach->last_edge);
  tach->last_edge = now;

  return IRQ_HANDLED;
}



#pragma mark - Probe/remove

/**
 * Teardown tachs, unregistering IRQs
 */
static void thermal_teardown_tachs(struct thermal *self)
{
  int tach_num;
  for (tach_num = 0; tach_num < THERMAL_NUM_TACH_SIGNALS; tach_num++) {
    if (self->tachs[tach_num].irq_num >= 0) {
      struct tach_channel *tach = &self->tachs[tach_num];
      disable_irq(tach->irq_num);
      free_irq(tach->irq_num, (void *)tach);
    }
  }
}

/**
 * Initialize tach structures, setting interrupt handlers
 */
static int thermal_init_tachs(struct thermal *self)
{
  int err = 0;
  int tach_num;
  /* init tachs struct to safe values */
  for (tach_num = 0; tach_num < THERMAL_NUM_TACH_SIGNALS; tach_num++) {
    self->tachs[tach_num].irq_num = -1;
  }
  /* allocate interrupts */
  for (tach_num = 0; tach_num < THERMAL_NUM_TACH_SIGNALS; tach_num++) {
    int pin_id = tach_pin_ids[tach_num];
    int gpio = self->gpios[pin_id];
    int irq_num = gpio_to_irq(gpio);
    struct tach_channel *tach = &self->tachs[tach_num];
    int irq;
    const struct pin_config *pin = &thermal_pin_configs[pin_id];

    tach->last_period = ktime_set(0,0);

    if (unlikely(irq_num < 0)) {
      pr_err("no IRQ for GPIO pin \"%s\": %d\n", pin->name, irq_num);
      err = -ENOENT;
      goto error;
    }

    tach->irq_num = irq_num;
    irq = request_irq(irq_num, (irq_handler_t)tach_irq_handler,
        IRQF_TRIGGER_RISING, pin->name, (void*)tach);
    if (unlikely(irq < 0)) {
      pr_err("no IRQ for IRQ number %d: %d\n", irq_num, irq);
      err = -ENOENT;
      goto error;
    }
  }

error:
  return err;
}



#pragma mark - Heater PWM

static ktime_t duty_to_ktime(u16 duty_fraction)
{
  if (duty_fraction == 0 || duty_fraction == HEATER_DUTY_MAX) {
    /* Return maximum period for 0% and 100% duty cycles. */
    return heater_pwm_period;
  } else {
    /* Otherwise, return the appropriate fraction of the maximum period. */
    return ns_to_ktime((ktime_to_ns(heater_pwm_period)*duty_fraction) >> HEATER_DUTY_CYCLE_BITS);
  }
}


/* Called from hard interrupt context */
static enum hrtimer_restart heater_pwm_timer_cb(struct hrtimer *timer)
{
  struct thermal *self = container_of(timer, struct thermal, heater_pwm_timer);
  u16 duty_fraction = self->heater_duty_fraction;
  int gpio = self->gpios[PIN_WATER_HEATER];
  int new_gpio_level = 0;

  if (duty_fraction == 0 || duty_fraction == HEATER_DUTY_MAX) {
    /* 0% - turn heater off  */
    /* 100% - turn heater on */
    new_gpio_level = (duty_fraction != 0);
  } else {
    /* Are we setting the heater GPIO output high or low? */
    /* Read the current level and invert it. */
    new_gpio_level = !gpio_get_value(gpio);
    if (!new_gpio_level) {
      /* On the falling edge, calculate off-time instead of on-time */
      duty_fraction = HEATER_DUTY_MAX-duty_fraction;
    }
  }
  gpio_set_value(gpio, new_gpio_level);
  hrtimer_forward_now(timer, duty_to_ktime(duty_fraction));
  return HRTIMER_RESTART;
}


static void thermal_set_heater_duty_fraction(struct thermal *self, u16 duty_fraction)
{
  /* Will update heater output on the next timer cb */
  self->heater_duty_fraction = duty_fraction;
}



#pragma mark -

int thermal_probe(struct platform_device *pdev)
{
  struct thermal *self;
  int ret = 0;
  if (!thermal_enabled) { dev_info(&pdev->dev, "%s: disabled, skipping", __func__); return 0; }
  dev_info(&pdev->dev, "%s: started", __func__);

  /* Allocate driver data */
  self = devm_kzalloc(&pdev->dev, sizeof(*self), GFP_KERNEL);
  if (!self) {
    return -ENOMEM;
  }
  self->dev = &pdev->dev;
  platform_set_drvdata(pdev, self);

  hrtimer_init(&self->heater_pwm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  self->heater_pwm_timer.function = heater_pwm_timer_cb;

  /* Set up GPIOs */
  ret = io_init_gpios(pdev->dev.of_node, thermal_pin_configs, self->gpios, THERMAL_NUM_GPIO_PINS);
  if (ret) {
    goto failed_io_init;
  }

  /* Set up PWMs */
  ret = io_init_pwms(pdev->dev.of_node, thermal_pwm_configs, self->pwms, THERMAL_NUM_PWM_CHANNELS);
  if (ret) {
    goto failed_pwm_init;
  }

  /* Set up tach interrupts */
  ret = thermal_init_tachs(self);
  if (ret) {
    goto failed_tach_init;
  }

  /* Create sysfs attributes */
  ret = sysfs_create_group(&pdev->dev.kobj, &thermal_attr_group);
  if (ret < 0) {
    dev_err(&pdev->dev, "failed to register attribute group");
    goto failed_create_group;
  }

  /* Add a link in /sys/glowforge */
  ret = sysfs_create_link(glowforge_kobj, &pdev->dev.kobj, THERMAL_GROUP_NAME);
  if (ret) {
    goto failed_create_link;
  }

  /* Add deadman switch notifier */
  self->dms_notifier.notifier_call = thermal_dms_handler;
  dms_notifier_chain_register(&dms_notifier_list, &self->dms_notifier);

  /* Set heater off, call timer immediately */
  thermal_set_heater_duty_fraction(self, 0);
  hrtimer_start(&self->heater_pwm_timer, ktime_zero, HRTIMER_MODE_REL);

  dev_info(&pdev->dev, "%s: done", __func__);
  return 0;

failed_create_link:
  sysfs_remove_group(&pdev->dev.kobj, &thermal_attr_group);
failed_create_group:
  io_release_pwms(self->pwms, THERMAL_NUM_PWM_CHANNELS);
failed_tach_init:
  thermal_teardown_tachs(self);
failed_pwm_init:
  io_release_gpios(self->gpios, THERMAL_NUM_GPIO_PINS);
failed_io_init:
  return ret;
}


int thermal_remove(struct platform_device *pdev)
{
  struct thermal *self = platform_get_drvdata(pdev);
  if (!thermal_enabled) { return 0; }
  dev_info(&pdev->dev, "%s: started", __func__);
  thermal_make_safe(self);
  dms_notifier_chain_unregister(&dms_notifier_list, &self->dms_notifier);
  hrtimer_cancel(&self->heater_pwm_timer);
  sysfs_remove_link(&pdev->dev.kobj, THERMAL_GROUP_NAME);
  sysfs_remove_group(&pdev->dev.kobj, &thermal_attr_group);
  io_release_pwms(self->pwms, THERMAL_NUM_PWM_CHANNELS);
  thermal_teardown_tachs(self);
  io_release_gpios(self->gpios, THERMAL_NUM_GPIO_PINS);
  dev_info(&pdev->dev, "%s: done", __func__);
  return 0;
}
