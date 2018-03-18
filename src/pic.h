/**
 * pic.h
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
 *
 * The PIC presents itself as a series of 16-bit "registers" that represent
 * sensor values or output levels. Each transaction is exactly 3 bytes long.
 * A write is one command byte (with the high bit set) followed by two data
 * bytes in litle-endian order. A read is one command byte (with the high bit
 * clear) followed by two dummy bytes.
 *
 * This module provides several userspace APIs:
 *
 * /sys/glowforge/pic/raw
 *   Write to this file to send a chunk of raw binary data to the PIC. The
 *   number of bytes written must be a multiple of 3.
 *   Read from this file to obtain the binary data transmitted by the PIC
 *   during the previous write transaction.
 *
 * /sys/glowforge/pic/hex
 *   Similar to raw, except the input and output is in ASCII hexadecimal.
 *   For example, to write a set of registers:
 *     echo 18=0123,19=4567,1a=89ab,1b=cdef > /sys/glowforge/pic/hex
 *     The string must be a comma-separated list of register=value pairs.
 *   To read a set of registers:
 *     echo 18,19,1a,1b > /sys/glowforge/pic/hex && cat /sys/glowforge/pic/hex
 *     The string must be a comma-separated list of register numbers.
 *   Reading from this file returns the register values transmitted by the PIC
 *     during the previous write transaction. The string is a comma-separated
 *     list of register values, each exactly 4 hex characters long.
 *
 * /sys/glowforge/pic/lid_temp, /sys/glowforge/pic/step_current_x, etc.
 * (see pic_api.c for full list)
 *   Individual register access. Input and output is in ASCII decimal.
 *
 * /sys/glowforge/pic/grp_*
 *   Register groups. These allow reading or updating multiple registers at
 *   once. Input/output is a sequence of 16-bit binary little-endian values.
 *   When writing to a register group file, the number of bytes written must
 *   exactly match the number of registers in the group times 2. (because each
 *   register is 2 bytes in size.)
 */

#ifndef KERNEL_SRC_PIC_H_
#define KERNEL_SRC_PIC_H_

#include <linux/spi/spi.h>

int pic_probe(struct spi_device *spi);

int pic_remove(struct spi_device *spi);

#endif
