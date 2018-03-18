/**
 * pic.c
 *
 * SPI driver for the PIC that acts as an analog I/O controller.
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
#include "uapi/glowforge.h"
#include "notifiers.h"

extern struct kobject *glowforge_kobj;

/** Module parameters */
extern int pic_enabled;

/**
 * If 1, verify that each write to a register succeeded by reading back
 * the value.
 */
#define VERIFY_WRITES 1

/**
 * If 1, check the presence of the PIC by reading 16 bits from register 0
 * and comparing it with the magic value.
 */
#define ENABLE_DEVICE_DETECTION 1

/**
 * Microseconds to wait between each byte transfer.
 */
#define TRANSFER_DELAY_USECS  2

/**
 * Set of initial values to at initialization time.
 */
static const struct pic_transaction initial_values[] = {
  { REGISTER_WRITE_COMMAND(PIC_REG_STEP_DAC_X), 33 },
  { REGISTER_WRITE_COMMAND(PIC_REG_STEP_DAC_Y), 5 }
};


int pic_read_one_register(struct spi_device *spi, enum pic_register reg)
{
  struct pic *self = spi_get_drvdata(spi);
  struct spi_transfer x = {0};
  int ret;
  int status;

  x.len = sizeof(struct pic_transaction);
  x.delay_usecs = TRANSFER_DELAY_USECS;
  x.tx_buf = self->txbuf;
  x.rx_buf = self->rxbuf;

  mutex_lock(&self->lock);
  self->txbuf[0].cmd = REGISTER_READ_COMMAND(reg);
  self->rxbuf_size = x.len;
  status = spi_sync_transfer(spi, &x, 1);
  ret = (status == 0) ? self->rxbuf[0].value : status;
  mutex_unlock(&self->lock);
  return ret;
}


int pic_write_one_register(struct spi_device *spi, enum pic_register reg, pic_value new_value)
{
  struct pic *self = spi_get_drvdata(spi);
  struct spi_transfer x = {0};
  int ret;

  x.len = sizeof(struct pic_transaction);
  x.tx_buf = self->txbuf;
  x.delay_usecs = TRANSFER_DELAY_USECS;
#if VERIFY_WRITES
  x.len *= 2;
  x.rx_buf = self->rxbuf;
#endif

  mutex_lock(&self->lock);
  self->txbuf[0].cmd = REGISTER_WRITE_COMMAND(reg);
  self->txbuf[0].value = new_value;
#if VERIFY_WRITES
  self->txbuf[1].cmd = REGISTER_READ_COMMAND(reg);
#endif
  self->rxbuf_size = x.len;
  ret = spi_sync_transfer(spi, &x, 1);

#if VERIFY_WRITES
  if (ret == 0) {
    if (new_value == self->rxbuf[1].value) {
      ret = 0;
    } else {
      dev_err(&spi->dev, "attempted to write value %d to register %d but read back %d", new_value, reg, self->rxbuf[1].value);
      ret = -EIO;
    }
  }
#endif
  mutex_unlock(&self->lock);
  return ret;
}


int pic_read_register_range(struct spi_device *spi, enum pic_register first_reg, enum pic_register last_reg, char *buf)
{
  struct pic *self = spi_get_drvdata(spi);
  struct spi_transfer x = {0};
  int ret;
  int num_regs;
  int i;
  pic_value *bptr = (pic_value *)buf;

  if (first_reg > last_reg) {
    return -EINVAL;
  }

  num_regs = last_reg-first_reg+1;
  x.len = sizeof(struct pic_transaction)*num_regs;
  x.delay_usecs = TRANSFER_DELAY_USECS;
  x.tx_buf = self->txbuf;
  x.rx_buf = self->rxbuf;

  mutex_lock(&self->lock);
  for (i = 0; i < num_regs; i++) {
    self->txbuf[i].cmd = REGISTER_READ_COMMAND(first_reg+i);
  }
  self->rxbuf_size = x.len;
  ret = spi_sync_transfer(spi, &x, 1);
  if (ret == 0) {
    for (i = 0; i < num_regs; i++) {
      *bptr++ = self->rxbuf[i].value;
    }
  }
  mutex_unlock(&self->lock);
  return (ret == 0) ? num_regs*sizeof(pic_value) : ret;
}


int pic_write_register_range(struct spi_device *spi, enum pic_register first_reg, enum pic_register last_reg, const char *buf, size_t count)
{
  struct pic *self = spi_get_drvdata(spi);
  struct spi_transfer x = {0};
  int ret;
  int num_regs;
  int i;
  pic_value *bptr = (pic_value *)buf;

  if (first_reg > last_reg) {
    return -EINVAL;
  }

  num_regs = last_reg-first_reg+1;
  if (count != num_regs*sizeof(pic_value)) {
    dev_err(&spi->dev, "%d bytes required to write %d registers, but received %d", num_regs*sizeof(pic_value), num_regs, count);
    return -EINVAL;
  }

  x.len = sizeof(struct pic_transaction)*num_regs;
  x.tx_buf = self->txbuf;
  x.delay_usecs = TRANSFER_DELAY_USECS;
#if VERIFY_WRITES
  x.len *= 2;
  x.rx_buf = self->rxbuf;
#endif

  mutex_lock(&self->lock);
  for (i = 0; i < num_regs; i++) {
    self->txbuf[i].cmd = REGISTER_WRITE_COMMAND(first_reg+i);
    self->txbuf[i].value = *bptr++;
  }
#if VERIFY_WRITES
  for (i = 0; i < num_regs; i++) {
    self->txbuf[num_regs+i].cmd = REGISTER_READ_COMMAND(first_reg+i);
  }
#endif
  self->rxbuf_size = x.len;
  ret = spi_sync_transfer(spi, &x, 1);
#if VERIFY_WRITES
  if (ret == 0) {
    bptr = (pic_value *)buf;
    for (i = 0; i < num_regs; i++) {
      pic_value expected = *bptr++;
      pic_value actual = self->rxbuf[num_regs+i].value;
      if (actual != expected) {
        dev_err(&spi->dev, "attempted to write value %d to register %d but read back %d", expected, first_reg+i, actual);
        ret = -EIO;
        break;
      }
    }
  }
#endif
  mutex_unlock(&self->lock);
  return (ret == 0) ? count : ret;
}


int pic_copy_last_read_data(struct spi_device *spi, void *buf)
{
  struct pic *self = spi_get_drvdata(spi);
  int ret;
  mutex_lock(&self->lock);
  memcpy(buf, self->rxbuf, self->rxbuf_size);
  ret = self->rxbuf_size;
  mutex_unlock(&self->lock);
  return ret;
}


int pic_write_data(struct spi_device *spi, const void *buf, size_t count)
{
  struct pic *self = spi_get_drvdata(spi);
  struct spi_transfer x = {0};
  int ret;

  if (count % sizeof(struct pic_transaction) != 0) { return -EINVAL; }
  if (count > MAX_REGISTERS_PER_TRANSFER*sizeof(struct pic_transaction)) { return -E2BIG; }

  x.len = count;
  x.delay_usecs = TRANSFER_DELAY_USECS;
  x.tx_buf = self->txbuf;
  x.rx_buf = self->rxbuf;

  mutex_lock(&self->lock);
  memcpy(self->txbuf, buf, count);
  self->rxbuf_size = count;
  ret = spi_sync_transfer(spi, &x, 1);
  mutex_unlock(&self->lock);
  return ret;
}


/* Deadman switch trip notification */
static void pic_make_safe(struct spi_device *spi)
{
  /* Zero out everything */
  static pic_value zeros[PIC_NUM_OUTPUT_REGISTERS] = {0};
  pic_write_register_range(spi, PIC_FIRST_OUTPUT_REGISTER, PIC_LAST_OUTPUT_REGISTER, (char *)zeros, sizeof(zeros));
  dev_err(&spi->dev, "making safe");
}


static int pic_dms_handler(struct notifier_block *nb, unsigned long action, void *data)
{
  struct pic *self = container_of(nb, struct pic, dms_notifier);
  struct spi_device *spi = to_spi_device(self->dev);
  pic_make_safe(spi);
  return 0;
}


int pic_probe(struct spi_device *spi)
{
  struct pic *self;
  int ret;
  if (!pic_enabled) { dev_info(&spi->dev, "%s: disabled, skipping", __func__); return 0; }
  dev_info(&spi->dev, "%s: started", __func__);
  dev_dbg(&spi->dev, "SPI bus %hd, cs %hhd, %d Hz, %d bits/word, mode 0x%hx",
    spi->master->bus_num,
    spi->chip_select,
    spi->max_speed_hz,
    spi->bits_per_word,
    spi->mode);

  /* Allocate driver data */
  self = devm_kzalloc(&spi->dev, sizeof(*self), GFP_KERNEL);
  if (!self) {
    return -ENOMEM;
  }
  self->txbuf = devm_kmalloc(&spi->dev, TX_RX_BUF_SIZE, GFP_KERNEL|GFP_DMA);
  if (!self->txbuf) {
    return -ENOMEM;
  }
  /* Set rxbuf to point to the second half of the buffer */
  self->rxbuf = &(self->txbuf[PIC_NUM_REGISTERS*2]);
  self->dev = &spi->dev;
  mutex_init(&self->lock);
  spi_set_drvdata(spi, self);

#if ENABLE_DEVICE_DETECTION
  /* Detect the device */
  ret = pic_read_one_register(spi, PIC_REG_DUMMY);
  if (ret < 0) {
    dev_err(&spi->dev, "failed to communicate with device, error %d", ret);
    goto failed_pic_detect;
  }
  if (ret != PIC_MAGIC_NUMBER) {
    dev_err(&spi->dev, "failed to detect device: returned ID %04hx, expected %04hx", ret, PIC_MAGIC_NUMBER);
    ret = -ENODEV;
    goto failed_pic_detect;
  } else {
    dev_dbg(&spi->dev, "found device, ID=%04hx", ret);
  }
#endif

  /* Register the LED devices */
  ret = pic_register_leds(spi);
  if (ret < 0) {
    dev_err(&spi->dev, "failed to register LEDs");
    goto failed_register_leds;
  }

  /* Create sysfs attributes */
  ret = sysfs_create_group(&spi->dev.kobj, &pic_attr_group);
  if (ret < 0) {
    dev_err(&spi->dev, "failed to register attribute group");
    goto failed_create_group;
  }

  /* Add a link in /sys/glowforge */
  ret = sysfs_create_link(glowforge_kobj, &spi->dev.kobj, PIC_GROUP_NAME);
  if (ret) {
    goto failed_create_link;
  }

  /* Add deadman switch notifier */
  self->dms_notifier.notifier_call = pic_dms_handler;
  dms_notifier_chain_register(&dms_notifier_list, &self->dms_notifier);

  /* Set initial values */
  pic_write_data(spi, initial_values, sizeof(initial_values));

  dev_info(&spi->dev, "%s: done", __func__);
  return 0;

failed_create_link:
  sysfs_remove_group(&spi->dev.kobj, &pic_attr_group);
failed_create_group:
  pic_unregister_leds(spi);
failed_register_leds:
failed_pic_detect:
  mutex_destroy(&self->lock);
  return ret;
}


int pic_remove(struct spi_device *spi)
{
  struct pic *self = spi_get_drvdata(spi);
  if (!pic_enabled) { return 0; }
  dev_info(&spi->dev, "%s: started", __func__);
  pic_make_safe(spi);
  dms_notifier_chain_unregister(&dms_notifier_list, &self->dms_notifier);
  sysfs_remove_link(&spi->dev.kobj, THERMAL_GROUP_NAME);
  sysfs_remove_group(&spi->dev.kobj, &pic_attr_group);
  pic_unregister_leds(spi);
  mutex_destroy(&self->lock);
  dev_info(&spi->dev, "%s: done", __func__);
  return 0;
}
