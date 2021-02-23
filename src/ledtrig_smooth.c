/**
 * ledtrig_smooth.c
 *
 * LED trigger that supports smooth fading between values.
 * Compensates for nonlinear brightness using a gamma curve.
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
 *
 * Smooth transitions are achieved by simulating LED brightness with a
 * critically-damped mass-spring system.
 *
 * Five attributes are exposed:
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
 * - dither: (0,1,2,3,4)
 *   Amount of temporal dithering to apply during fade animations.
 *   Due to the nonlinearity of LEDs, fades at the lower end of the brightness
 *   scale may look choppy. Setting this attribute to a nonzero value enables
 *   temporal dithering (i.e. rapidly flickering back and forth between two
 *   brightness levels at various duty cycles), giving the appearance of more
 *   brightness resolution and smoother fades.
 *   Increasing this value adds more intermediate brightness levels (i.e. duty
 *   cycle values) but also increases the refresh rate during fades, and thus
 *   CPU usage. Temporal dithering is never applied when the brightness is at a
 *   steady state. (i.e. not fading)
 *   Default is 0 (off).
 */

#include <linux/device.h>
#include <linux/slab.h>
#include "../uapi/glowforge.h"
#include "tasklet_hrtimer_compat.h"
#include "ledtrig_compat.h"
#include "ktime_compat.h"

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define ZERO_LOWER_BITS(i,b) ((i) & (~((1<<(b))-1)))



#pragma mark - Defines/tables

#define MSECS_PER_UPDATE        20

/* Temporal dithering modes. */
/* Higher values of temporal dithering provide more intermediate levels */
/* of brightness, which can result in smoother fades at the low end of the */
/* brightness range. However, higher dithering values require higher */
/* refresh rates during animation. */
#define MAX_DITHER_BITS   4
struct dither_mode_params {
  const unsigned int dither_bits;
  const unsigned int dither_frac_mask;
  const ktime_t timestep_interval;
};

#define DITHER_MODE_PARAMS(b) {(b), (1<<(b))-1, NSEC_TO_KTIME((MSECS_PER_UPDATE*NSEC_PER_MSEC)/(1<<(b)))}
static const struct dither_mode_params dither_modes[MAX_DITHER_BITS+1] = {
  DITHER_MODE_PARAMS(0), /* No intermediate steps between brightness values; 50 Hz refresh rate */
  DITHER_MODE_PARAMS(1), /* 1 intermediate step between brightness values; 100 Hz refresh rate */
  DITHER_MODE_PARAMS(2), /* 3 intermediate steps between brightness values; 200 Hz refresh rate */
  DITHER_MODE_PARAMS(3), /* 7 intermediate steps between brightness values; 400 Hz refresh rate */
  DITHER_MODE_PARAMS(4)  /* 15 intermediate steps between brightness values; 800 Hz refresh rate */
};


#define CORRECTION_TABLE_FRAC_BITS  8
static unsigned int correction_table[256] = {
  /* Gamma 2.2, in 10.8 fixed point: correction_table[i] = floor((1023 * (i/256)^2.2) * 256) */
  0, 1, 6, 14, 28, 45, 68, 96, 128, 167, 210, 259, 314, 375, 441, 514, 592, 677, 767, 864, 968, 1077, 1194, 1316, 1446, 1581, 1724, 1873, 2029, 2192, 2362, 2539, 2723, 2913, 3111, 3316, 3528, 3747, 3974, 4207, 4448, 4697, 4953, 5216, 5486, 5764, 6050, 6343, 6644, 6952, 7268, 7592, 7923, 8262, 8609, 8964, 9326, 9697, 10075, 10461, 10855, 11257, 11667, 12085, 12511, 12946, 13388, 13838, 14297, 14763, 15238, 15721, 16212, 16712, 17220, 17736, 18260, 18793, 19334, 19884, 20442, 21008, 21583, 22166, 22758, 23358, 23967, 24584, 25210, 25845, 26488, 27140, 27801, 28470, 29147, 29834, 30529, 31233, 31946, 32668, 33398, 34137, 34885, 35642, 36408, 37182, 37966, 38758, 39560, 40370, 41189, 42018, 42855, 43701, 44557, 45421, 46295, 47177, 48069, 48970, 49879, 50798, 51727, 52664, 53611, 54566, 55531, 56506, 57489, 58482, 59484, 60495, 61516, 62546, 63585, 64634, 65692, 66759, 67836, 68922, 70017, 71122, 72237, 73361, 74494, 75637, 76789, 77951, 79123, 80303, 81494, 82694, 83904, 85123, 86352, 87590, 88838, 90096, 91363, 92640, 93927, 95223, 96529, 97845, 99170, 100505, 101850, 103205, 104570, 105944, 107328, 108722, 110125, 111539, 112962, 114396, 115839, 117291, 118754, 120227, 121710, 123202, 124705, 126217, 127739, 129272, 130814, 132366, 133928, 135501, 137083, 138675, 140278, 141890, 143512, 145145, 146788, 148440, 150103, 151776, 153459, 155152, 156855, 158569, 160292, 162026, 163770, 165524, 167288, 169063, 170847, 172642, 174448, 176263, 178089, 179925, 181771, 183627, 185494, 187371, 189259, 191156, 193065, 194983, 196912, 198851, 200800, 202760, 204731, 206711, 208702, 210704, 212716, 214738, 216771, 218814, 220868, 222932, 225007, 227092, 229188, 231294, 233410, 235538, 237675, 239823, 241982, 244152, 246332, 248522, 250723, 252935, 255157, 257390, 259633, 261888
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

/* precondition: a <= b */
static inline unsigned int lerp(unsigned int a, unsigned int b, unsigned int fpart) {
  return a + (((b-a) * fpart) >> FRAC_BITS);
}



#pragma mark -
/* Timestep, in seconds */
static fixed dt = FIXED(0, MSECS_PER_UPDATE*0.001);

/* Threshold for equality comparisons */
static fixed epsilon = FIXED(0, 0.125);

struct ledtrig_smooth_data {
  struct led_classdev *parent_dev;
  SOFTIRQ_HRTIMER timer;
  
  /* Parameters */
  fixed target;
  fixed c; /* damping constant; spring constant is derived */
  int pulse_on_count; /* multiples of MSECS_PER_UPDATE */
  int pulse_off_count; /* multiples of MSECS_PER_UPDATE */
  int dither_bits; /* level of temporal dithering */
  struct dither_mode_params current_dither_mode;
  
  /* Simulation state */
  fixed value;
  fixed velocity;
  fixed accel;
  int dither_counter;
  int pulse_counter;
  unsigned int raw_value;
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


static unsigned int value_to_brightness(const fixed value, const fixed endpoint)
{
  /* Interpolate between two entries in the correction table. */
  unsigned int lower_index = CLAMP(value.ipart, 0, 255);
  unsigned int upper_index = CLAMP(lower_index+1, 0, 255);
  /* lower should always be <= upper, because the correction_table values are */
  /* monotonically increasing */
  unsigned int lower = correction_table[lower_index];
  unsigned int upper = correction_table[upper_index];
  /* If one of the entries is an endpoint, round it to an integer value. */
  /* Prevents a noticeable brightness discontinuity when the simulation ends, */
  /* i.e. the temporal dithering stops. */
  if (lower_index == endpoint.ipart) {
    /* round down */
    lower = ZERO_LOWER_BITS(lower, CORRECTION_TABLE_FRAC_BITS);
  }
  if (upper_index == endpoint.ipart) {
    /* round up */
    upper = ZERO_LOWER_BITS(upper, CORRECTION_TABLE_FRAC_BITS) + (1 << CORRECTION_TABLE_FRAC_BITS);
  }
  return lerp(lower, upper, value.fpart);
}


static void update_led_brightness(struct ledtrig_smooth_data *data)
{
  struct led_classdev *led_cdev = data->parent_dev;
  unsigned int max_brightness = led_cdev->max_brightness;
  /* dither between raw_value and raw_value+1 */
  unsigned int integer_brightness = data->raw_value >> CORRECTION_TABLE_FRAC_BITS;
  if (((data->raw_value >> (CORRECTION_TABLE_FRAC_BITS-data->current_dither_mode.dither_bits)) & data->current_dither_mode.dither_frac_mask) > data->dither_counter) {
    integer_brightness = min(integer_brightness+1, max_brightness);
  }
  led_set_brightness(led_cdev, integer_brightness);
}


static enum hrtimer_restart timestep(struct hrtimer *timer)
{
  struct ledtrig_smooth_data *data = container_of(TO_SOFTIRQ_HRTIMER(timer), struct ledtrig_smooth_data, timer);
  bool pulsing = update_pulser(data);
  bool done = false;
  fixed k, offset, fspring, fdamping_neg, new_accel, half_v;

  data->dither_counter = (data->dither_counter + 1) & data->current_dither_mode.dither_frac_mask;
  /* On non-update frames, don't need to update the simulation, just the dithering */
  if (data->dither_counter != 0) {
    update_led_brightness(data);
    hrtimer_forward_now(timer, data->current_dither_mode.timestep_interval);
    return HRTIMER_RESTART;
  }

  /* For a critically damped system (zeta=1), c/(2*sqrt(m*k)) = 1. */
  /* If m=1 (as in this case), then k = (c^2)/4. */
  /* https://en.wikipedia.org/wiki/Damping#Critical_damping_.28.CE.B6_.3D_1.29 */
  k = fixdiv4(fixmul(data->c, data->c));
 
  /* Fspring = k * (target - value[t]) */
  offset = fixsub(data->target, data->value);
  fspring = fixmul(offset, k);
  /* -Fdamping = c * v[t] */
  fdamping_neg = fixmul(data->c, data->velocity);
  /* a[t+dt] = Fspring - (-Fdamping) (mass = 1, no division necessary) */
  new_accel = fixsub(fspring, fdamping_neg);

  /* Velocity Verlet integration. */
  /* https://en.wikipedia.org/wiki/Verlet_integration#Velocity_Verlet */
  /* Compute intermediate term: v[t] + (1/2)*a[t]*dt */
  half_v = fixadd(data->velocity, fixmul(fixdiv2(data->accel), dt));
  /* x[t+dt] = x[t] + halfV*dt */
  /* (expanded: x[t+dt] = x[t] + v[t]*dt + (1/2)*a[t]*dt^2) */
  data->value = fixadd(data->value, fixmul(half_v, dt));
  /* v[t+dt] = halfV + (1/2)*a[t+dt]*dt */
  /* (expanded: v[t+dt] = v[t] + (1/2)*(a[t] + a[d+dt])*dt) */
  data->velocity = fixadd(half_v, fixmul(fixdiv2(new_accel), dt));
  data->accel = new_accel;

  /* Have we hit the target value? */
  if (fixeq(data->target, data->value, epsilon)) {
    done = true;
    data->value = data->target;
  }
  data->raw_value = value_to_brightness(data->value, data->target);
  update_led_brightness(data);
  
  /* Once we hit the target value, we can stop iterating. */
  if (done && !pulsing) {
    data->velocity = FIXED_INT(0);
    data->accel = FIXED_INT(0);
    data->dither_counter = 0;
    return HRTIMER_NORESTART;
  } else {
    hrtimer_forward_now(timer, data->current_dither_mode.timestep_interval);
    return HRTIMER_RESTART;
  }
}


static inline void start_fade(struct ledtrig_smooth_data *data) {
  /* wake up the simulation */
  SOFTIRQ_HRTIMER_START(&data->timer, data->current_dither_mode.timestep_interval);
}


fixed value_from_raw_brightness(int raw_brightness)
{
  /* Binary search to find the index of the given 10-bit brightness value, or */
  /* the index of the next-highest value. */
  /* Can't use bsearch() because it only finds exact matches. */
  /* The following code is adapted from linux/lib/bsearch.c. */
  size_t start = 0, end = sizeof(correction_table)/sizeof(correction_table[0]);
  int result, mid = 0;
  raw_brightness <<= CORRECTION_TABLE_FRAC_BITS;
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

static ssize_t set_dither_mode(struct ledtrig_smooth_data *data, unsigned int mode) {
  if (mode > MAX_DITHER_BITS) { return -EINVAL; }
  memcpy(&data->current_dither_mode, &dither_modes[mode], sizeof(struct dither_mode_params));
  return 0;
}


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
  start_fade(data);
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
  start_fade(data);
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
    start_fade(data);
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
    start_fade(data);
  }
  return size;
}


static ssize_t dither_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  return sprintf(buf, "%d\n", data->current_dither_mode.dither_bits);
}


static ssize_t dither_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  unsigned long new_value;
  ssize_t ret = kstrtoul(buf, 10, &new_value);
  if (ret) { return ret; }
  ret = set_dither_mode(data, new_value);
  return (ret) ? ret : size;
}


static ssize_t crossfading_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  bool pulsing = (data->pulse_on_count && data->pulse_off_count);
  bool converged = fixeq(data->target, data->value, epsilon);
  char crossfading = (pulsing || !converged) ? LED_CROSSFADING_STATE_CROSSFADING : LED_CROSSFADING_STATE_STATIC;
  return sprintf(buf, "%d\n", crossfading);
}


static ssize_t crossfading_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  // crossfading is not writable, but DEVICE_ATTR requires a function with this signature.
  return 0;
}


static DEVICE_ATTR(target, 0644, target_show, target_store);
static DEVICE_ATTR(speed, 0644, speed_show, speed_store);
static DEVICE_ATTR(pulse_on, 0644, pulse_on_show, pulse_on_store);
static DEVICE_ATTR(pulse_off, 0644, pulse_off_show, pulse_off_store);
static DEVICE_ATTR(dither, 0644, dither_show, dither_store);
static DEVICE_ATTR(crossfading, 0444, crossfading_show, crossfading_store);
static struct attribute *smooth_attrs[] = {
  &dev_attr_target.attr,
  &dev_attr_speed.attr,
  &dev_attr_pulse_on.attr,
  &dev_attr_pulse_off.attr,
  &dev_attr_dither.attr,
  &dev_attr_crossfading.attr,
  NULL
};
const struct attribute_group smooth_attr_group = { .attrs = smooth_attrs };



#pragma mark - Activate/deactivate

static LEDTRIG_ACTIVATE_RETURN_TYPE smooth_activate(struct led_classdev *led_cdev)
{
  struct ledtrig_smooth_data *data = kzalloc(sizeof(struct ledtrig_smooth_data), GFP_KERNEL);
  if (!data) {
    return LEDTRIG_ACTIVATE_RETURN_ERR;
  }
  data->parent_dev = led_cdev;
  
  if (sysfs_create_group(&led_cdev->dev->kobj, &smooth_attr_group)) {
    goto failed_create_group;
  }
 
  /* A reverse lookup in the correction table is required to set the current */
  /* value from the LED's current raw brightness value */
  set_dither_mode(data, 0);
  data->value = value_from_raw_brightness(led_cdev->brightness);
  data->target = data->value;
  data->raw_value = value_to_brightness(data->value, data->target);
  data->c = FIXED_INT(16);
  led_cdev->trigger_data = data;
  SOFTIRQ_HRTIMER_INIT(&data->timer, timestep, CLOCK_MONOTONIC);
  led_cdev->activated = true;
  return LEDTRIG_ACTIVATE_RETURN_OK;
  
failed_create_group:
  kfree(data);
  return LEDTRIG_ACTIVATE_RETURN_ERR;
}


static void smooth_deactivate(struct led_classdev *led_cdev)
{
  struct ledtrig_smooth_data *data = led_cdev->trigger_data;
  if (led_cdev->activated) {
    SOFTIRQ_HRTIMER_CANCEL(&data->timer);
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
