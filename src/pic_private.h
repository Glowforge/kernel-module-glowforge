/**
 * pic_private.h
 *
 * Private header for the PIC driver.
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

#ifndef KERNEL_SRC_PIC_PRIVATE_H_
#define KERNEL_SRC_PIC_PRIVATE_H_

#include "pic.h"
#include "notifiers.h"
#include "uapi/glowforge.h"
#include <linux/leds.h>

#define REGISTER_READ_COMMAND(reg)  ((reg)<<1)
#define REGISTER_WRITE_COMMAND(reg) (0x80|((reg)<<1))

struct pic_transaction {
  pic_command cmd;
  pic_value value;
} __attribute__((packed));

/**
 * Maximum number of registers that can be transferred at once.
 * We allocate enough to write the entire register set and read it back.
 */
#define MAX_REGISTERS_PER_TRANSFER  (PIC_NUM_REGISTERS*2)

/**
 * Size of the unified transmit/receive buffer.
 * We allocate double the amount of space required to write and read the entire
 * register set; the first half is used as the transmit buffer, and the second
 * half is used as the receive buffer.
 */
#define TX_RX_BUF_SIZE              (MAX_REGISTERS_PER_TRANSFER*2*sizeof(struct pic_transaction))

/**
 * Private data members.
 */
struct pic {
  /** Device that owns this data */
  struct device *dev;
  /** Lock to ensure mutually exclusive access */
  struct mutex lock;
  /** DMA-safe buffers for transfers */
  struct pic_transaction *txbuf;
  struct pic_transaction *rxbuf;
  /** Size of the most recent transfer */
  size_t rxbuf_size;
  /** Notifiers */
  struct notifier_block dms_notifier;
  /** LED devices */
  struct led_classdev lid_led;
  struct led_classdev button_led_1;
  struct led_classdev button_led_2;
  struct led_classdev button_led_3;
};


/* defined in pic_api.c */
extern const struct attribute_group pic_attr_group;

/* defined in pic.c */
int pic_read_one_register(struct spi_device *spi, enum pic_register reg);
int pic_write_one_register(struct spi_device *spi, enum pic_register reg, pic_value new_value);
int pic_read_register_range(struct spi_device *spi, enum pic_register first_reg, enum pic_register last_reg, char *buf);
int pic_write_register_range(struct spi_device *spi, enum pic_register first_reg, enum pic_register last_reg, const char *buf, size_t count);
int pic_copy_last_read_data(struct spi_device *spi, void *buf);
int pic_write_data(struct spi_device *spi, const void *buf, size_t count);

/* defined in pic_leds.c */
int pic_register_leds(struct spi_device *spi);
void pic_unregister_leds(struct spi_device *spi);

#endif
