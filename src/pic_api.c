/**
 * pic_api.c
 *
 * PIC userspace API handlers.
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
#include "device_attr.h"
#include "uapi/glowforge.h"

static ssize_t pic_read_register_ascii(struct device *dev, enum pic_register reg, char *buf)
{
  struct spi_device *spi = to_spi_device(dev);
  ssize_t value = pic_read_one_register(spi, reg);
  if (value >= 0) {
    return scnprintf(buf, PAGE_SIZE, "%hu\n", value);
  } else {
    return value;
  }
}


static ssize_t pic_write_register_ascii(struct device *dev, enum pic_register reg, const char *buf, size_t count)
{
  struct spi_device *spi = to_spi_device(dev);
  pic_value new_value = 0;
  int ret;
  /* Error if the entire buffer is used; we need a null-terminator */
  if (count >= PAGE_SIZE) { return -E2BIG; }

  if (sscanf(buf, "%hu", &new_value) != 1) {
    return -EINVAL;
  }
  ret = pic_write_one_register(spi, reg, new_value);
  return (ret >= 0) ? count : ret;
}


/**
 * Returns the last chunk of data received over SPI.
 */
static ssize_t raw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  return pic_copy_last_read_data(to_spi_device(dev), buf);
}


/**
 * Writes a chunk of data over SPI. Data length must be an even multiple of 3
 * and must be less than the preallocated buffer size.
 */
static ssize_t raw_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  int ret;
  if (count == 0) { return 0; }
  ret = pic_write_data(to_spi_device(dev), buf, count);
  return (ret == 0) ? count : ret;
}


static ssize_t hex_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  /* Copy the raw bytes into a temporary buffer */
  struct pic_transaction raw[MAX_REGISTERS_PER_TRANSFER];
  int num_bytes = pic_copy_last_read_data(to_spi_device(dev), raw);
  int num_regs = num_bytes / sizeof(struct pic_transaction);
  char *buf_start = buf;
  int i;

  /* Write their hex representations into the output string */
  for (i = 0; i < num_regs; i++) {
    if (i != 0) { *buf++ = ','; };
    buf += scnprintf(buf, 5, "%04hx", raw[i].value);
  }
  *buf++ = '\n';
  return buf - buf_start;
}


static ssize_t hex_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  struct pic_transaction raw[MAX_REGISTERS_PER_TRANSFER];
  int num_tx = 1;
  int i;
  int nfields;
  pic_command reg;
  pic_value value;
  int ret;

  if (count == 0) { return 0; };
  /* Error if the entire buffer is used; we need a null-terminator */
  if (count >= PAGE_SIZE) { return -E2BIG; }

  /* Count the number of commas to determine how many transactions are being requested */
  for (i = 0; i < count; i++) {
    num_tx += (buf[i] == ',');
  }
  if (num_tx > MAX_REGISTERS_PER_TRANSFER) { return -E2BIG; }

  /* Scan each command */
  /* Note: the string in buf is null-terminated by the kernel. */
  for (i = 0; i < num_tx && buf; i++) {
    nfields = sscanf(buf, "%hhx=%hx", &reg, &value);
    if (nfields == 2) {
      raw[i].cmd = REGISTER_WRITE_COMMAND(reg);
      raw[i].value = value;
    } else if (nfields == 1) {
      raw[i].cmd = REGISTER_READ_COMMAND(reg);
      raw[i].value = 0;
    } else {
      return -EINVAL;
    }
    buf = strchr(buf, ','); /* advance to the next comma */
    if (buf) { buf++; } /* skip past the comma */
  }

  ret = pic_write_data(to_spi_device(dev), raw, i*sizeof(struct pic_transaction));
  return (ret == 0) ? count : ret;
}



#define DEFINE_READONLY_REG_ATTR(name, reg) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    return pic_read_register_ascii(dev, reg, buf); } \
  static DEVICE_ATTR(name, S_IRUSR, name##_show, NULL)

#define DEFINE_READWRITE_REG_ATTR(name, reg) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    return pic_read_register_ascii(dev, reg, buf); } \
  static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
    return pic_write_register_ascii(dev, reg, buf, count); } \
  static DEVICE_ATTR(name, S_IRUSR|S_IWUSR, name##_show, name##_store)

#define DEFINE_READONLY_RANGE_ATTR(name, first_reg, last_reg) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    return pic_read_register_range(to_spi_device(dev), first_reg, last_reg, buf); } \
  static DEVICE_ATTR(name, S_IRUSR, name##_show, NULL)

#define DEFINE_READWRITE_RANGE_ATTR(name, first_reg, last_reg) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    return pic_read_register_range(to_spi_device(dev), first_reg, last_reg, buf); } \
  static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
    return pic_write_register_range(to_spi_device(dev), first_reg, last_reg, buf, count); } \
  static DEVICE_ATTR(name, S_IRUSR|S_IWUSR, name##_show, name##_store)

/** Single register attributes (ASCII decimal values) */
DEFINE_READONLY_REG_ATTR(ATTR_PIC_ID, PIC_REG_DUMMY);
DEFINE_READONLY_REG_ATTR(ATTR_WATER_TEMP_1, PIC_REG_WATER_THERM1);
DEFINE_READONLY_REG_ATTR(ATTR_WATER_TEMP_2, PIC_REG_WATER_THERM2);
DEFINE_READONLY_REG_ATTR(ATTR_TEC_TEMP, PIC_REG_TEC_THERM);
DEFINE_READONLY_REG_ATTR(ATTR_PWR_TEMP, PIC_REG_PWR_THERM);
DEFINE_READONLY_REG_ATTR(ATTR_LID_IR_1, PIC_REG_LID_IR_DET1);
DEFINE_READONLY_REG_ATTR(ATTR_LID_IR_2, PIC_REG_LID_IR_DET2);
DEFINE_READONLY_REG_ATTR(ATTR_LID_IR_3, PIC_REG_LID_IR_DET3);
DEFINE_READONLY_REG_ATTR(ATTR_LID_IR_4, PIC_REG_LID_IR_DET4);
DEFINE_READONLY_REG_ATTR(ATTR_HV_CURRENT, PIC_REG_HV_ISENSE);
DEFINE_READONLY_REG_ATTR(ATTR_HV_VOLTAGE, PIC_REG_HV_VSENSE);
DEFINE_READWRITE_REG_ATTR(ATTR_X_CURRENT, PIC_REG_STEP_DAC_X);
DEFINE_READWRITE_REG_ATTR(ATTR_Y_CURRENT, PIC_REG_STEP_DAC_Y);
DEFINE_READWRITE_REG_ATTR(ATTR_LID_LED, PIC_REG_LID_LED);
DEFINE_READWRITE_REG_ATTR(ATTR_BUTTON_LED_1, PIC_REG_BUTTON_LED1);
DEFINE_READWRITE_REG_ATTR(ATTR_BUTTON_LED_2, PIC_REG_BUTTON_LED2);
DEFINE_READWRITE_REG_ATTR(ATTR_BUTTON_LED_3, PIC_REG_BUTTON_LED3);

/** Register group attributes (16-bit little-endian binary values) */
DEFINE_READONLY_RANGE_ATTR(ATTR_PIC_ALL, PIC_FIRST_REGISTER, PIC_LAST_REGISTER);
DEFINE_READONLY_RANGE_ATTR(ATTR_PIC_SENSORS, PIC_FIRST_SENSOR_REGISTER, PIC_LAST_SENSOR_REGISTER);
DEFINE_READONLY_RANGE_ATTR(ATTR_PIC_HV, PIC_FIRST_HV_REGISTER, PIC_LAST_HV_REGISTER);
DEFINE_READWRITE_RANGE_ATTR(ATTR_PIC_OUTPUTS, PIC_FIRST_OUTPUT_REGISTER, PIC_LAST_OUTPUT_REGISTER);
DEFINE_READWRITE_RANGE_ATTR(ATTR_PIC_BUTTON_LEDS, PIC_FIRST_BUTTON_LED_REGISTER, PIC_LAST_BUTTON_LED_REGISTER);

/** Allows reading/writing arbitrary register sequences. */
DEFINE_DEVICE_ATTR(ATTR_PIC_RAW, S_IRUSR|S_IWUSR, raw_show, raw_store);
DEFINE_DEVICE_ATTR(ATTR_PIC_HEX, S_IRUSR|S_IWUSR, hex_show, hex_store);


static struct attribute *pic_attrs[] = {
  DEV_ATTR_PTR(ATTR_PIC_ID),
  DEV_ATTR_PTR(ATTR_WATER_TEMP_1),
  DEV_ATTR_PTR(ATTR_WATER_TEMP_2),
  DEV_ATTR_PTR(ATTR_TEC_TEMP),
  DEV_ATTR_PTR(ATTR_PWR_TEMP),
  DEV_ATTR_PTR(ATTR_LID_IR_1),
  DEV_ATTR_PTR(ATTR_LID_IR_2),
  DEV_ATTR_PTR(ATTR_LID_IR_3),
  DEV_ATTR_PTR(ATTR_LID_IR_4),
  DEV_ATTR_PTR(ATTR_HV_CURRENT),
  DEV_ATTR_PTR(ATTR_HV_VOLTAGE),
  DEV_ATTR_PTR(ATTR_X_CURRENT),
  DEV_ATTR_PTR(ATTR_Y_CURRENT),
  DEV_ATTR_PTR(ATTR_LID_LED),
  DEV_ATTR_PTR(ATTR_BUTTON_LED_1),
  DEV_ATTR_PTR(ATTR_BUTTON_LED_2),
  DEV_ATTR_PTR(ATTR_BUTTON_LED_3),
  DEV_ATTR_PTR(ATTR_PIC_ALL),
  DEV_ATTR_PTR(ATTR_PIC_SENSORS),
  DEV_ATTR_PTR(ATTR_PIC_HV),
  DEV_ATTR_PTR(ATTR_PIC_OUTPUTS),
  DEV_ATTR_PTR(ATTR_PIC_BUTTON_LEDS),
  DEV_ATTR_PTR(ATTR_PIC_RAW),
  DEV_ATTR_PTR(ATTR_PIC_HEX),
  NULL
};

const struct attribute_group pic_attr_group = {
  .attrs = pic_attrs
};
