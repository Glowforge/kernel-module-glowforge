/**
 * cnc_buffer.c
 *
 * Manages the queue of pulse data shared with the SDMA engine.
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
#include "cnc_private.h"
#include "sdma_macros.h"

/**
 * Size of the contiguous memory region used as the pulse data buffer.
 * Must be a power of two.
 */
#define CNC_BUFFER_SIZE     (128 * SZ_1M)

/**
 * Enforce a minimum gap between head and tail.
 * This allows a small amount of data to be retained after it has been
 * processed.
 */
#define CNC_BUFFER_GAP_SIZE (32 * SZ_1K)


int cnc_buffer_init(struct cnc *self)
{
  self->pulsebuf_total_bytes = 0;
  self->pulsebuf_size = CNC_BUFFER_SIZE;
  self->pulsebuf_virt = dma_alloc_coherent(self->dev, self->pulsebuf_size, &self->pulsebuf_phys, GFP_DMA|GFP_KERNEL);
  if (!self->pulsebuf_virt) {
    dev_err(self->dev, "unable to allocate coherent buffer of size %u", self->pulsebuf_size);
    return -ENOMEM;
  }
  return kfifo_init(&self->pulsebuf_fifo, self->pulsebuf_virt, self->pulsebuf_size);
}


void cnc_buffer_destroy(struct cnc *self)
{
  if (self->pulsebuf_virt && self->pulsebuf_phys && self->pulsebuf_size) {
    dma_free_coherent(self->dev, self->pulsebuf_size, self->pulsebuf_virt, self->pulsebuf_phys);
  }
}


/**
 * Synchronizes the current head value from the SDMA engine to the kfifo.
 * Should be called before any operation that requires querying the amount of
 * free space in the fifo.
 * Performed by cnc_buffer_is_empty() and cnc_buffer_add_user_data().
 * May sleep.
 * Returns nonzero if there was an error reading the SDMA channel context.
 */
static int cnc_buffer_sync_head(struct cnc *self)
{
  int ret = sdma_get_reg(self->sdmac, &self->pulsebuf_fifo.kfifo.out, scratch4);
  if (ret) {
    dev_err(self->dev, "context fetch failed: %d", ret);
  }
  return ret;
}


ssize_t cnc_buffer_add_user_data(struct cnc *self, const uint8_t __user *data, size_t count)
{
  unsigned int copied;

  /* read current head value from SDMA */
  int ret = cnc_buffer_sync_head(self);
  if (ret) { return ret; }

  /* bail if there's not enough room */
  if (kfifo_avail(&self->pulsebuf_fifo) < count+CNC_BUFFER_GAP_SIZE) {
    return -ENOMEM;
  }

  /* copy userspace data into fifo; */
  /* entire buffer must fit, don't allow partial copies */
  ret = kfifo_from_user(&self->pulsebuf_fifo, data, count, &copied);
  if (ret) { return ret; }
  if (copied != count) { return -ENOMEM; }

  self->pulsebuf_total_bytes += count;
  /* inform SDMA of the new tail index; just change one register */
  ret = sdma_set_reg(self->sdmac, &self->pulsebuf_fifo.kfifo.in, scratch5);
  if (ret) {
    dev_err(self->dev, "context load failed: %d", ret);
    return ret;
  }
  return count;
}


/* Only clears the necessary registers in the SDMA context, */
/* leaving everything else (its working registers, program counter, etc) */
/* alone. You should be able to call this while the SDMA engine is running. */
int cnc_buffer_clear(struct cnc *self, unsigned int flags)
{
  bool clear_data = false;
  uint32_t regs_to_clear = 0;
  int first_reg, last_reg;
  struct sdma_context_data cleared_context = {{0}};

  /* Determine which registers we need to clear. */
  /* (note: we can only clear a contiguous region) */
  if (flags & CNC_BUFFER_CLEAR_DATA) {
    clear_data = true;
    regs_to_clear |= (1 << sdma_reg_number(scratch3))  /* byte count */
                  |  (1 << sdma_reg_number(scratch4))  /* head */
                  |  (1 << sdma_reg_number(scratch5)); /* tail */
  }
  if (flags & CNC_BUFFER_CLEAR_POSITION) {
    regs_to_clear |= (1 << sdma_reg_number(scratch0))  /* X */
                  |  (1 << sdma_reg_number(scratch1))  /* Y */
                  |  (1 << sdma_reg_number(scratch2)); /* Z */
  }
  if (regs_to_clear == 0) { return -EINVAL; }

  /* Find the range of registers to clear */
  /* (basically, take the union of the desired ranges) */
  /* note: ffs()/fls() return values are 1-indexed */
  first_reg = ffs(regs_to_clear)-1;
  last_reg = fls(regs_to_clear)-1;

  if (clear_data) {
    kfifo_reset(&self->pulsebuf_fifo);
    /* script needs the new head/tail indexes (they might not be 0) */
    cleared_context.scratch4 = self->pulsebuf_fifo.kfifo.out;
    cleared_context.scratch5 = self->pulsebuf_fifo.kfifo.in;
    self->pulsebuf_total_bytes = 0;
  }

  /* convert register numbers (i.e. word offsets) to byte offsets */
  return sdma_load_partial_context(self->sdmac,
    (struct sdma_context_data *)(((uint32_t *)(&cleared_context))+first_reg), /* source byte pointer */
    first_reg*sizeof(uint32_t), /* destination byte offset */
    (last_reg-first_reg+1)*sizeof(uint32_t)); /* byte count */
}

/* may sleep */
int cnc_buffer_is_empty(struct cnc *self)
{
  cnc_buffer_sync_head(self);
  return kfifo_is_empty(&self->pulsebuf_fifo);
}

uint32_t cnc_buffer_max_backtrack_length(struct cnc *self)
{
  return min(self->pulsebuf_total_bytes,
             kfifo_size(&self->pulsebuf_fifo)-CNC_BUFFER_GAP_SIZE);
}

uint32_t cnc_buffer_fifo_bitmask(struct cnc *self)
{ return self->pulsebuf_fifo.kfifo.mask; }

uint32_t cnc_buffer_fifo_start_phys(struct cnc *self)
{ return self->pulsebuf_phys; }

uint32_t cnc_buffer_total_bytes(struct cnc *self)
{ return self->pulsebuf_total_bytes; }