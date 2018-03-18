/**
 * sdma_macros.h
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
#ifndef KERNEL_SRC_SDMA_MACROS_H_
#define KERNEL_SRC_SDMA_MACROS_H_

#include <linux/platform_data/dma-imx-sdma.h>

/**
 * Helper macros for getting/setting registers in a channel context.
 * These may sleep.
 */
#define sdma_get_regs(s,b,r,n) sdma_fetch_partial_context(s, b, offsetof(struct sdma_context_data, r), n)
#define sdma_set_regs(s,b,r,n) sdma_load_partial_context(s, (struct sdma_context_data *)(b), offsetof(struct sdma_context_data, r), n)
#define sdma_get_reg(s,b,r) sdma_get_regs(s, b, r, sizeof(uint32_t))
#define sdma_set_reg(s,b,r) sdma_set_regs(s, b, r, sizeof(uint32_t))
#define sdma_reg_number(r)  (offsetof(struct sdma_context_data, r) / sizeof(uint32_t))

#define sdma_context_address_for_channel(ch) (0x800 + (sizeof(struct sdma_context_data)/4)*(ch))

#endif
