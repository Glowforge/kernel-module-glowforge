/**
 * ledtrig_smooth.c
 *
 * LED trigger that supports smooth fading between values.
 * Compensates for nonlinear brightness using a gamma curve.
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
 *
 * Smooth transitions are achieved by simulating LED brightness with a
 * critically-damped mass-spring system.
 *
 * Four attributes are exposed:
 * - target: (range: [0, 255])
 *   The new brightness set-point. The LED fades from its current value to the
 *   target value. May be changed while the LED is already fading.
 * - speed: (range: [0, 160])
 *   The speed at which the LED seeks its target brightness.
 *   Technically, this is controlling the mass-spring system's damping constant.
 *   (The spring constant is derived from this value.)
 *   The default is 64.
 * - pulse_on:  (milliseconds)
 * - pulse_off: (milliseconds)
 *   When both values are > 0, the LED's target will alternate between minimum
 *   and maximum brightness automatically.
 *   The delay between target=255 and target=0 is specified by pulse_on.
 *   The delay between target=0 and target=255 is specified by pulse_off.
 *   Both values are internally truncated to multiples of MSECS_PER_UPDATE.
 *
 * Please don't let Linus see this. I can't even imagine what he'd say if he
 * found out I'm doing physics calculations with hand-rolled fixed-point math
 * routines in kernel space.
 */

#include <linux/device.h>
#include <linux/leds.h>
#include <linux/slab.h>

#define MSECS_PER_UPDATE  20

static unsigned int correction_table[256] = {
  /* Gamma 2.2: correction_table[i] = 1023 * (i/256)^2.2 */
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 27, 28, 29, 30, 32, 33, 35, 36, 37, 39, 40, 42, 43, 45, 47, 48, 50, 52, 54, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73, 75, 77, 79, 82, 84, 86, 88, 91, 93, 96, 98, 100, 103, 106, 108, 111, 113, 116, 119, 122, 124, 127, 130, 133, 136, 139, 142, 145, 148, 151, 154, 157, 160, 164, 167, 170, 174, 177, 180, 184, 187, 191, 194, 198, 202, 205, 209, 213, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 269, 273, 277, 282, 286, 290, 295, 299, 304, 309, 313, 318, 323, 327, 332, 337, 342, 347, 351, 356, 361, 366, 371, 377, 382, 387, 392, 397, 403, 408, 413, 419, 424, 430, 435, 441, 446, 452, 458, 463, 469, 475, 481, 487, 493, 498, 504, 510, 517, 523, 529, 535, 541, 547, 554, 560, 566, 573, 579, 586, 592, 599, 606, 612, 619, 626, 632, 639, 646, 653, 660, 667, 674, 681, 688, 695, 702, 710, 717, 724, 731, 739, 746, 754, 761, 769, 776, 784, 792, 799, 807, 815, 823, 830, 838, 846, 854, 862, 870, 878, 887, 895, 903, 911, 920, 928, 936, 945, 953, 962, 970, 979, 988, 996, 1005, 1014, 1023
};



#pragma mark - Fixed point math

/**
 * Signed fixed point data type.
 * Floating point math is forbidden in the kernel,
 * and GCC's fixed-point math extensions can't be used because kernel modules
 * don't link against libgcc.
 * So we need to roll our own data type.
 */
#define INT_BITS  20
#define FRAC_BITS 12
typedef union {
  struct {
    unsigned int fpart:FRAC_BITS;
    signed int ipart:INT_BITS;
  };
  s32 intval;
} fixed;

#define FIXED_INT(i)  (fixed){ .ipart=(i), .fpart=0 }
#define FIXED(i,f)    (fixed){ .ipart=(i), .fpart=(f)*(1<<FRAC_BITS) } /* 0 <= f < 1 */

static inline fixed fixadd(fixed a, fixed b) {
  return (fixed)(a.intval + b.intval);
}

static inline fixed fixsub(fixed a, fixed b) {
  return (fixed)(a.intval - b.intval);
}

static inline fixed fixmul(fixed a, fixed b) {
  s64 quot = (((s64)a.intval * (s64)b.intval) >> FRAC_BITS);
  return (fixed)((s32)(quot));
}

static inline fixed fixdiv2(fixed a) {
  return (fixed)(a.intval >> 1);
}

static inline fixed fixdiv4(fixed a) {
  return (fixed)(a.intval >> 2);
}

static inline bool fixeq(fixed a, fixed b, fixed tolerance) {
  return (abs(a.intval - b.intval) <= tolerance.intval);
}



#pragma mark -
/* Timestep, in seconds */
static fixed dt = FIXED(0, MSECS_PER_UPDATE*0.001);

/* Threshold for equality comparisons */
static fixed epsilon = FIXED(0, 0.0625);

struct ledtrig_smooth_data {
  struct timer_list timer;
  
  /* Parameters */
  fixed target;
  fixed c; /* damping constant; spring constant is derived */
  int pulse_on_count; /* multiples of MSECS_PER_UPDATE */
  int pulse_off_count; /* multiples of MSECS_PER_UPDATE */
  
  /* Simulation state */
  fixed value;
  fixed velocity;
  fixed accel;
  int pulse_counter;
};



#pragma mark - Simulation

static bool update_pulser(struct ledtrig_smooth_data *data)
{
  bool pulsing = (data->pulse_on_count && data->pulse_off_count);
  if (!pulsing) { return false; }

  if (data->pulse_counter == 0) {
    data->target = FIXED_INT(255);
  } else if (data->pulse_counter == data->pulse_on_count) {
    data->target = FIXED_INT(0);
    data->pulse_counter = -data->pulse_off_count;
  }
  data->pulse_counter++;
  return true;
}


static bool set_pulse_periods(struct ledtrig_smooth_data *data, int on, int off)
{
  /* When setting on/off to smaller values, */
  /* need to make sure to update the current count to prevent a wraparound, */
  /* or a period that's too long */
  if (on && data->pulse_counter > on) {
    data->pulse_counter = on;
  } else if (off && data->pulse_counter < -off) {
    data->pulse_counter = -(off-1);
  }
  data->pulse_on_count = on;
  data->pulse_off_count = off;
  if (!on || !off) { data->pulse_counter = 0; }
  return (on && off);
}


static void timestep(struct led_classdev *led_cdev)
{
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  bool pulsing = update_pulser(data);
  s16 clamped_value;
  
  /* For a critically damped system (zeta=1), c/(2*sqrt(m*k)) = 1. */
  /* If m=1 (as in this case), then k = (c^2)/4. */
  /* https://en.wikipedia.org/wiki/Damping#Critical_damping_.28.CE.B6_.3D_1.29 */
  fixed k = fixdiv4(fixmul(data->c, data->c));
 
  /* Fspring = k * (target - value[t]) */
  fixed offset = fixsub(data->target, data->value);
  fixed fspring = fixmul(offset, k);
  /* -Fdamping = c * v[t] */
  fixed fdamping_neg = fixmul(data->c, data->velocity);
  /* a[t+dt] = Fspring - (-Fdamping) (mass = 1, no division necessary) */
  fixed new_accel = fixsub(fspring, fdamping_neg);

  /* Velocity Verlet integration. */
  /* https://en.wikipedia.org/wiki/Verlet_integration#Velocity_Verlet */
  /* Compute intermediate term: v[t] + (1/2)*a[t]*dt */
  fixed half_v = fixadd(data->velocity, fixmul(fixdiv2(data->accel), dt));
  /* x[t+dt] = x[t] + halfV*dt */
  /* (expanded: x[t+dt] = x[t] + v[t]*dt + (1/2)*a[t]*dt^2) */
  data->value = fixadd(data->value, fixmul(half_v, dt));
  /* v[t+dt] = halfV + (1/2)*a[t+dt]*dt */
  /* (expanded: v[t+dt] = v[t] + (1/2)*(a[t] + a[d+dt])*dt) */
  data->velocity = fixadd(half_v, fixmul(fixdiv2(new_accel), dt));
  data->accel = new_accel;
  
  clamped_value = data->value.ipart;
  if (clamped_value < 0) {
    clamped_value = 0;
  } else if (clamped_value > 255) {
    clamped_value = 255;
  }
  led_set_brightness(led_cdev, correction_table[clamped_value]);
  
  /* Once we hit the target value, we can stop iterating. */
  if (fixeq(data->target, data->value, epsilon) && !pulsing) {
    data->target = data->value;
    data->velocity = FIXED_INT(0);
    data->accel = FIXED_INT(0);
  } else {
    mod_timer(&data->timer, jiffies + msecs_to_jiffies(MSECS_PER_UPDATE));
  }
}


fixed value_from_raw_brightness(int raw_brightness)
{
  /* Binary search to find the index of the given 10-bit brightness value, or */
  /* the index of the next-highest value. */
  /* Can't use bsearch() because it only finds exact matches. */
  /* The following code is adapted from linux/lib/bsearch.c. */
  size_t start = 0, end = sizeof(correction_table)/sizeof(correction_table[0]);
  int result, mid = 0;
  while (start < end) {
    mid = start + (end - start) / 2;
    result = raw_brightness - correction_table[mid];
    if (result < 0) { end = mid; }
    else if (result > 0) { start = mid+1; }
    else { break; }
  }
  return FIXED_INT(mid);
}



#pragma mark - API

static ssize_t target_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  return sprintf(buf, "%hd\n", data->target.ipart);
}


static ssize_t target_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  unsigned long new_target;
  ssize_t ret = kstrtoul(buf, 10, &new_target);
  if (ret) { return ret; }
  if (new_target > 255) { return -EINVAL; }
  data->target = FIXED_INT(new_target);
  timestep(led_cdev); /* wake up the simulation */
  return size;
}


/* To allow users to select from a better range of fade speeds, */
/* the "speed" value exposed via sysfs is shifted to the right. */
/* Thus, "speed" is a 6.2 fixed-point unsigned integer. */
#define SPEED_C_SHIFT 2

/* The maximum value that can be written to the "speed" attribute. */
/* Values higher than this are likely to cause an arithmetic overflow in the */
/* simulation due to the limited range of fixed-point arithmetic. */
#define SPEED_MAX     160

static ssize_t speed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  u8 speed = data->c.ipart << SPEED_C_SHIFT;
  return sprintf(buf, "%hhd\n", speed);
}


static ssize_t speed_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  unsigned long new_speed;
  ssize_t ret = kstrtoul(buf, 10, &new_speed);
  if (ret) { return ret; }
  if (new_speed == 0 || new_speed > SPEED_MAX) { return -EINVAL; }
  data->c.intval = new_speed << (FRAC_BITS-SPEED_C_SHIFT);
  timestep(led_cdev); /* wake up the simulation */
  return size;
}


static ssize_t pulse_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  return sprintf(buf, "%d\n", data->pulse_on_count*MSECS_PER_UPDATE);
}


static ssize_t pulse_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  unsigned long new_on;
  ssize_t ret = kstrtoul(buf, 10, &new_on);
  if (ret) { return ret; }
  if (set_pulse_periods(data, new_on/MSECS_PER_UPDATE, data->pulse_off_count)) {
    timestep(led_cdev);
  }
  return size;
}


static ssize_t pulse_off_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  return sprintf(buf, "%d\n", data->pulse_off_count*MSECS_PER_UPDATE);
}


static ssize_t pulse_off_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  unsigned long new_off;
  ssize_t ret = kstrtoul(buf, 10, &new_off);
  if (ret) { return ret; }
  if (set_pulse_periods(data, data->pulse_on_count, new_off/MSECS_PER_UPDATE)) {
    timestep(led_cdev);
  }
  return size;
}


static DEVICE_ATTR(target, 0644, target_show, target_store);
static DEVICE_ATTR(speed, 0644, speed_show, speed_store);
static DEVICE_ATTR(pulse_on, 0644, pulse_on_show, pulse_on_store);
static DEVICE_ATTR(pulse_off, 0644, pulse_off_show, pulse_off_store);
static struct attribute *smooth_attrs[] = {
  &dev_attr_target.attr,
  &dev_attr_speed.attr,
  &dev_attr_pulse_on.attr,
  &dev_attr_pulse_off.attr,
  NULL
};
const struct attribute_group smooth_attr_group = { .attrs = smooth_attrs };



#pragma mark - Activate/deactivate

static void smooth_activate(struct led_classdev *led_cdev)
{
  struct ledtrig_smooth_data *data = kzalloc(sizeof(struct ledtrig_smooth_data), GFP_KERNEL);
  if (!data) {
    return;
  }
  
  if (sysfs_create_group(&led_cdev->dev->kobj, &smooth_attr_group)) {
    goto failed_create_group;
  }
 
  /* A reverse lookup in the correction table is required to set the current */
  /* value from the LED's current raw brightness value */
  data->value = value_from_raw_brightness(led_cdev->brightness);
  data->target = data->value;
  data->c = FIXED_INT(16);
  led_cdev->trigger_data = data;
  setup_timer(&data->timer, (void (*)(unsigned long))timestep, (unsigned long)led_cdev);
  led_cdev->activated = true;
  return;
  
failed_create_group:
  kfree(data);
}


static void smooth_deactivate(struct led_classdev *led_cdev)
{
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  if (led_cdev->activated) {
    del_timer_sync(&data->timer);
    sysfs_remove_group(&led_cdev->dev->kobj, &smooth_attr_group);
    kfree(data);
    led_cdev->activated = false;
  }
}


static struct led_trigger smooth_trigger = {
  .name       = "smooth",
  .activate   = smooth_activate,
  .deactivate = smooth_deactivate,
};


int ledtrig_smooth_init(void)
{
  return led_trigger_register(&smooth_trigger);
}


void ledtrig_smooth_remove(void)
{
  led_trigger_unregister(&smooth_trigger);
}
