/**
 * cnc.c
 *
 * Drives the stepper motors and laser.
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
 * The step and direction pins are controlled by an SDMA script;
 * this subsystem is responsible for overseeing the operation of
 * the SDMA engine, the periodic interval timer (EPIT) that drives it,
 * and the GPIO port settings.
 */

#include "cnc_private.h"
#include "io.h"
#include "notifiers.h"
#include "sdma_macros.h"

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>

/** Module parameters */
extern int cnc_enabled;

/* needs to be exposed because imx6 GPIO doesn't provide get_direction() */
struct gpio_desc {
  struct gpio_chip *chip;
  unsigned long flags;
#ifdef CONFIG_DEBUG_FS
  const char *label;
#endif
};
#define FLAG_IS_OUT 1


/**
 * If 1, the module starts up in the DISABLED state when it's loaded, and does
 * not enable the 40V supply.
 * If 0, the module starts up in the IDLE state when it's loaded, and enables
 * the 40V supply.
 */
#define INITIAL_STATE_DISABLED  1

/** Minimum, maximum, and default step frequencies. */
#define STEP_FREQUENCY_MIN      1000
#define STEP_FREQUENCY_MAX      200000
#define STEP_FREQUENCY_DEFAULT  10000

/** Number of bits of PWM resolution */
#define LASER_PWM_BITS          7
/** Laser power duty cycle when idle */
#define LASER_PWM_IDLE_DUTY     65535

/**
 * How often the charge pump input is pulsed while running
 * (to keep the laser firing)
 */
#define CHARGE_PUMP_INTERVAL_NS               (200 * NSEC_PER_MSEC)

/**
 * Minimum step frequency for controlled decelerations and accelerations.
 */
#define RAMP_UPDATE_INTERVAL_NS               (10 * NSEC_PER_MSEC)

/**
 * Minimum step frequency for controlled decelerations and accelerations.
 * (Deceleration stops when this frequency is reached, and acceleration begins
 * at this frequency.)
 */
#define RAMP_MIN_STEP_FREQUENCY               900

/**
 * During a controlled acceleration/deceleration, the factor by which the
 * current step frequency is increased/decreased every update.
 * Treated as an inverse power of 2, so 3 corresponds to
 *   freq +=/-= freq >> 3 (at every step, add/subtract 1/8th of orig. step freq)
 */
#define STEP_FREQUENCY_RAMP_FACTOR            3


static const struct pwm_channel_config laser_pwm_config = {
  "laser-pwm", BITS_TO_PERIOD_NS(LASER_PWM_BITS)
};

#define NUM_STEPPER_FAULT_SIGNALS 3

static const pin_id stepper_fault_gpios[NUM_STEPPER_FAULT_SIGNALS] = {
  [FAULT_X]  = PIN_X_FAULT,
  [FAULT_Y1] = PIN_Y1_FAULT,
  [FAULT_Y2] = PIN_Y2_FAULT
};

/**
 * Fatal fault conditions require the driver to stop immediately and enter the
 * FAULT state. If a non-fatal fault occurs, the driver will attept a controlled
 * deceleration and enter the IDLE state.
 */
static const u32 fatal_fault_conditions =
  (1 << FAULT_X) |
  (1 << FAULT_Y1) |
  (1 << FAULT_Y2);
#define FAULT_IS_FATAL(num) (fatal_fault_conditions & (1<<(num)))

#define FAULT_DEV_ID_FROM_CNC_AND_SIGNAL(dr, s) (void *)(((u32)dr) | (s & 3U))
#define CNC_FROM_FAULT_DEV_ID(dev_id) (struct cnc *)((u32)(dev_id) & (~3U))
#define SIGNAL_FROM_FAULT_DEV_ID(dev_id) ((u32)(dev_id) & 3U)

static const u32 sdma_script[] = {
  0x09010b00, 0x69c80400, 0x69c84e00, 0x7d6e50e7, 0x00bc52ef, 0x02bc00ca, 0x7d68009e, 0x68106209,
  0x02677d67, 0x50f7007f, 0x7c02124a, 0x02240353, 0x02617c01, 0x0333033c, 0x03520263, 0x7c02035c,
  0x03320331, 0x02667c01, 0x03516b2b, 0x080d7803, 0x01890189, 0x01890260, 0x7c010356, 0x02627c02,
  0x0354035d, 0x02657c01, 0x0355033e, 0x033f0264, 0x7c02035e, 0x035f5097, 0x03b06b2b, 0x08327803,
  0x01890189, 0x01890260, 0x7c0650c7, 0x02617c01, 0x20021801, 0x58c70262, 0x7c0650cf, 0x02637d01,
  0x20021801, 0x58cf0265, 0x7c0650d7, 0x02667d01, 0x20021801, 0x58d70121, 0x03360334, 0x033d0335,
  0x6b2b52f7, 0x50df009a, 0x58df50e7, 0x009a58e7, 0x50ff4800, 0x7d042001, 0x58ff7c01, 0x03000162,
  0x01227d93, 0x04000160, 0x7d8f0300, 0x04000160, 0x7d870161, 0x7de650f7, 0x007f7d06, 0x60d06dd3,
  0x3a7f6ac8, 0x68d30141, 0x01420160, 0x7dda0000
};

static const ktime_t ramp_update_interval_ktime = { .tv64 = RAMP_UPDATE_INTERVAL_NS };
static const ktime_t charge_pump_interval_ktime  = { .tv64 = CHARGE_PUMP_INTERVAL_NS };

extern struct kobject *glowforge_kobj;

static void _cnc_ramp_stop(struct cnc *self);
static void beam_detect_latch_reset(struct cnc *self);
static void toggle_charge_pump(struct cnc *self);

static int load_sdma_script(struct cnc *self)
{
  int ret;
  const u32 *script = sdma_script;
  size_t script_len = sizeof(sdma_script);
  /* set the script arguments and initial PC */
  /* see the specific asm file for argument requirements */
  struct sdma_context_data initial_context = {
    .channel_state = { .pc = self->sdma_script_origin * 2 },
    .gReg = {
      [4] = cnc_buffer_fifo_bitmask(self),
      [5] = io_pwm_sample_register_address(&self->laser_pwm),
      [6] = cnc_buffer_fifo_start_phys(self),
      [7] = sdma_context_address_for_channel(self->sdma_ch_num)
    },
    .pda = epit_status_register_address(self->epit),
    .mda = io_base_address(self->gpios, NUM_GPIO_PINS, cnc_sdma_pin_set),
    .ms = 0x00000000, /* source and destination address frozen; start in read mode */
    .ps = 0x000c0400, /* destination address frozen; 32-bit write size; start in write mode */
    .scratch6 = 1,    /* direction = forward */
  };

  /* write the test script code to RAM */
  /* don't use sdma_load_script() because the assembler output */
  /* is already in the correct endianness */
  dev_dbg(self->dev, "loading SDMA script (%d bytes)...", script_len);
  ret = sdma_write_datamem(self->sdma, (void *)script, script_len, self->sdma_script_origin);
  if (ret) {
    dev_err(self->dev, "failed to load script");
    return ret;
  }

  /* acquire the channel; it's triggered externally by the EPIT */
  sdma_setup_channel(self->sdmac, true);

  /* load the initial context */
  ret = sdma_load_partial_context(self->sdmac, &initial_context, 0, sizeof(initial_context));
  if (ret) {
    dev_err(self->dev, "failed to set up channel");
    return ret;
  }

  dev_dbg(self->dev, "script loaded");
  return ret;
}


int cnc_get_position(struct cnc *self, struct cnc_position *pos)
{
  /* Fetch current byte and head position from sdma engine */
  int ret = sdma_fetch_partial_context(self->sdmac, pos, offsetof(struct sdma_context_data, scratch0), sizeof(*pos));
  if (ret != 0) {
    return ret;
  }
  /* Splice in the total number of bytes enqueued */
  pos->bytes_total = cnc_buffer_total_bytes(self);
  return 0;
}


u32 cnc_get_step_frequency(struct cnc *self)
{
  return self->step_freq;
}


int cnc_set_step_frequency(struct cnc *self, u32 freq)
{
  int ret = 0;
  if (freq < STEP_FREQUENCY_MIN || freq > STEP_FREQUENCY_MAX) {
    return -ERANGE;
  }

  spin_lock_bh(&self->status_lock);
  /* Step frequency changes are forbidden while running */
  /* (this includes controlled acceleration/deceleration) */
  if (unlikely(self->status.state == STATE_RUNNING)) {
    ret = -EBUSY;
  } else {
    self->step_freq = freq;
    self->ramp_step_freq = freq;
    self->ramp_step_freq_delta = freq >> STEP_FREQUENCY_RAMP_FACTOR;
  }
  spin_unlock_bh(&self->status_lock);

  return ret;
}


/* Powers on the steppers without checking fault states */
static void stepper_power_on_unchecked(struct cnc *self)
{
  if (!regulator_is_enabled(self->supply_40v)) {
    if (regulator_enable(self->supply_40v)) {
      dev_err(self->dev, "unable to enable 40V supply");
    } else {
      dev_info(self->dev, "40V on");
    }
  }
  io_change_pins(self->gpios, NUM_GPIO_PINS, cnc_startup_pin_changes);
}


static int _stepper_power_on(struct cnc *self, int faults)
{
  /* Don't power on the steppers if the drivers are asserting a fault. */
  /* (It's possible that *enabling* the steppers and the 40V supply could */
  /* trigger a fault, but in that case, they'll be disabled immediately.) */
  if (faults & fatal_fault_conditions) {
    dev_err(self->dev, "driver(s) in fault state; not powering on");
    return -1;
  }
  stepper_power_on_unchecked(self);
  return 0;
}


/* acquires status_lock */
__maybe_unused static int stepper_power_on(struct cnc *self)
{
  return _stepper_power_on(self, cnc_triggered_faults(self));
}


static void stepper_power_off(struct cnc *self)
{
  if (regulator_is_enabled(self->supply_40v)) {
    if (regulator_disable(self->supply_40v)) {
      dev_err(self->dev, "unable to disable 40V supply");
    } else {
      dev_info(self->dev, "40V off");
    }
  }
  io_change_pins(self->gpios, NUM_GPIO_PINS, cnc_shutdown_pin_changes);
}


/* Must be called with status_lock held */
static void _driver_stop(struct cnc *self, enum cnc_state next_state)
{
  dev_dbg(self->dev, "stopping cut...");
  epit_stop(self->epit);
  hrtimer_cancel(&self->charge_pump_timer);
  sdma_event_disable(self->sdmac, epit_sdma_event(self->epit));
  _cnc_ramp_stop(self);

  /* If the next state is DISABLED, shut down the stepper drivers. */
  if (next_state == STATE_DISABLED) {
    stepper_power_off(self);
  }
  /* Otherwise, just ensure the laser and stepper lines are low */
  else {
    io_change_pins(self->gpios, NUM_GPIO_PINS, cnc_stop_pin_changes);
  }

  dev_dbg(self->dev, "stopped.");
  self->status.state = next_state;
  cnc_notify_state_changed(self);
}


/**
 * Common code for starting a controlled acceleration/deceleration.
 * Must be called with status_lock held when in kernel context.
 */
static void _cnc_ramp_start(struct cnc *self)
{
  /* Disable DMA control of laser enable; force the line low. */
  gpio_direction_input(self->gpios[PIN_LASER_ON]);
  /* Begin periodic updates */
  tasklet_hrtimer_start(&self->ramp_timer, ramp_update_interval_ktime,
    HRTIMER_MODE_REL);
}


/**
 * Stops a controlled acceleration/deceleration.
 * Must be called with status_lock held when in kernel context.
 */
static void _cnc_ramp_stop(struct cnc *self)
{
  /* If called by ramp_update_tasklet_fn, don't cancel the timer, because */
  /* we're already running in a tasklet and the kernel doesn't like that */
  if (!in_softirq()) {
    tasklet_hrtimer_cancel(&self->ramp_timer);
  }
  self->status.decelerating = false;
  self->status.accelerating = false;
}


/**
 * Starts a controlled deceleration.
 * Must be called with status_lock held when in kernel context.
 */
static void _cnc_decel_start(struct cnc *self)
{
  if (self->status.decelerating) {
    return;
  }
  dev_dbg(self->dev, "starting deceleration");
  if (!self->status.accelerating) {
    /* Don't suddenly jump the step frequency if we're already accelerating */
    self->ramp_step_freq = self->step_freq;
  }
  self->status.accelerating = false;
  self->status.decelerating = true;
  _cnc_ramp_start(self);
}


/**
 * Starts a controlled acceleration.
 * Must be called with status_lock held when in kernel context.
 */
static void _cnc_accel_start(struct cnc *self)
{
  if (self->status.accelerating) {
    return;
  }
  dev_dbg(self->dev, "starting acceleration");
  if (!self->status.decelerating) {
    /* Don't suddenly jump the step frequency if we're already decelerating */
    self->ramp_step_freq = RAMP_MIN_STEP_FREQUENCY;
  }
  self->status.accelerating = true;
  self->status.decelerating = false;
  _cnc_ramp_start(self);
}


/**
 * Controlled acceleration/deceleration update step.
 */
static enum hrtimer_restart ramp_update_tasklet_fn(struct hrtimer *timer)
{
  /* We don't need to protect the status field in this function, */
  /* because the tasklet won't ever run concurrently with itself or any other */
  /* tasklet (uniprocessor system, tasklets can't be preempted), and won't */
  /* run in the sections protected by spin_lock_bh()/spin_unlock_bh(). */
  struct tasklet_hrtimer *tasklet_hrtimer = container_of(timer, struct tasklet_hrtimer, timer);
  struct cnc *self = container_of(tasklet_hrtimer, struct cnc, ramp_timer);

  /* sanity check */
  if (!self->status.decelerating && !self->status.accelerating) {
    return HRTIMER_NORESTART;
  }

  if (self->status.decelerating) {
    if (self->ramp_step_freq <= RAMP_MIN_STEP_FREQUENCY) {
      dev_dbg(self->dev, "stopping deceleration");
      _driver_stop(self, STATE_IDLE);
      return HRTIMER_NORESTART;
    }
    self->ramp_step_freq -= self->ramp_step_freq_delta;
  }
  else if (self->status.accelerating) {
    self->ramp_step_freq += self->ramp_step_freq_delta;
    if (self->ramp_step_freq >= self->step_freq) {
      dev_dbg(self->dev, "stopping acceleration");
      epit_set_hz(self->epit, self->step_freq); /* restore full step freq */
      return HRTIMER_NORESTART;
    }
  }

  epit_set_hz(self->epit, self->ramp_step_freq);
  hrtimer_forward_now(timer, ramp_update_interval_ktime);
  return HRTIMER_RESTART;
}


/**
 * Called when the SDMA engine executes a "done 3" instruction, setting the
 * interrupt flag for our channel.
 * This callback executes in tasklet context.
 */
void cnc_sdma_interrupt(void *param)
{
  /* The "on_interrupt" flags modify the behavior of this IRQ handler. */
  struct cnc *self = (struct cnc *)param;
  spin_lock_bh(&self->status_lock);
  /* "enable_laser_on_interrupt" is set when we're expecting an SDMA waypoint */
  /* interrupt during a resume. */
  if (unlikely(self->status.enable_laser_on_interrupt)) {
    self->status.enable_laser_on_interrupt = false;
    gpio_direction_output(self->gpios[PIN_LASER_ON], 0);
  }
  /* "decel_on_interrupt" is set when we're expecting an SDMA waypoint */
  /* interrupt during a backtrack. */
  else if (unlikely(self->status.decel_on_interrupt)) {
    self->status.decel_on_interrupt = false;
    _cnc_decel_start(self);
  }
  /* Neither flag set; SDMA has reached the end of the data, stop the driver. */
  else {
    _driver_stop(self, STATE_IDLE);
  }
  spin_unlock_bh(&self->status_lock);
}


/**
 * cnc_run_with_options() needs a lot of arguments, so pack them all into
 * a 32-bit struct instead of passing them all individually.
 */
struct cnc_run_options {
  /** If != 0, SDMA will trigger a waypoint interrupt after this many steps. */
  unsigned int num_steps:28;
  /** 0 to run forward, 1 to run backward. */
  unsigned int backward:1;
  /** 0 to start at full speed, 1 to start with acceleration. */
  unsigned int accelerate:1;
  /** 0 to come to an immediate stop at end of data, 1 to decelerate. */
  unsigned int decelerate:1;
  /** 0 to reset laser power PWM duty cycle at end of data, 1 to preserve. */
  unsigned int preserve_power:1;
} __attribute__((packed));

/**
 * Used for run, backtrack, and resume.
 * Idle: start cutting if there is data
 * Running: do nothing
 * Disabled: enable the steppers and start cutting
 * Fault: do nothing (error state must be explicitly cleared)
 */
static int cnc_run_with_options(struct cnc *self, struct cnc_run_options opts)
{
  int ret = 0;
  bool need_to_start = false;
  spin_lock_bh(&self->status_lock);
  switch (self->status.state) {
    case STATE_RUNNING:
      ret = -EPERM;
      break;

    case STATE_FAULT:
    default:
      dev_err(self->dev, "cannot start in fault state");
      ret = -EPERM;
      break;

    case STATE_DISABLED:
    case STATE_IDLE:
      /* defer loading until we're out of atomic context */
      need_to_start = true;
      break;
  }
  spin_unlock_bh(&self->status_lock);

  if (need_to_start) {
    uint32_t regs[3];
    uint32_t num_steps = opts.num_steps;

    /* Ensure there is enough data enqueued. (may sleep) */
    if (cnc_buffer_is_empty(self)) {
      dev_err(self->dev, "cannot start cut; no data enqueued");
      return -ENODATA;
    }

    /* If backtracking, clamp num_steps to ensure we stop before hitting the */
    /* end of valid contiguous data */
    if (opts.backward) {
      num_steps = min(num_steps, cnc_buffer_max_backtrack_length(self));
      if (num_steps == 0) {
        /* Shouldn't happen; if cnc_buffer_max_backtrack_length() is 0 then */
        /* the cnc_buffer_is_empty() call would have returned true. num_steps */
        /* is also checked against 0 in cnc_backtrack(). Still, another check */
        /* doesn't hurt. (num_steps cannot be 0 during a backtrack, or else */
        /* we'll never get an interrupt to start deceleration.) */
        return -ENODATA;
      }
    }

    /* Set direction and interrupt point. */
    /* If processing backward, set scratch5 to ensure the DMA engine doesn't */
    /* go past the oldest data byte. Wraparound in subtraction is OK. */
    regs[0] /* scratch5 */ =
     (!opts.backward) ? self->pulsebuf_fifo.kfifo.in
                      : self->pulsebuf_fifo.kfifo.in-self->pulsebuf_total_bytes;
      /* Note when running backward: it would be incorrect to set scratch5 */
      /* to (self->pulsebuf_fifo.kfifo.in-num_steps). When head == tail, */
      /* the DMA engine will come to a dead stop. But when backtracking we */
      /* want to *decelerate* after num_steps, and only come to a dead stop */
      /* if we run out of room to backtrack. */
    regs[1] /* scratch6 */ = (!opts.backward) ? 0x00000001 : 0xFFFFFFFF;
    regs[2] /* scratch7 */ = num_steps;

    ret = sdma_set_regs(self->sdmac, regs, scratch5, sizeof(regs));
    if (ret) {
      dev_err(self->dev, "failed to set channel context");
      return ret;
    }

    /* We could have transitioned to FAULT between the start of the function */
    /* and now, so we have to lock and check the fault state. */
    /* If a fault occurs during the execution of this block, we'll get a */
    /* callback immediately after we release the lock, which will transition */
    /* the driver to the FAULT state. */
    spin_lock_bh(&self->status_lock);
    if (self->status.triggered_faults) {
      dev_err(self->dev, "attempt to start in fault state");
      ret = -EPERM;
    } else {
      bool enable_laser_on_interrupt = (opts.accelerate && !opts.backward);
      self->status.state = STATE_RUNNING;
      self->status.decel_on_interrupt = opts.decelerate;
      self->status.enable_laser_on_interrupt = enable_laser_on_interrupt;

      /* clear all fault conditions */
      self->status.triggered_faults &= fatal_fault_conditions;

      cnc_notify_state_changed(self);

      dev_dbg(self->dev, "starting cut...");
      /* Ensure the steppers are powered up */
      stepper_power_on_unchecked(self);
      if (!opts.preserve_power) {
        io_pwm_set_duty_cycle(&self->laser_pwm, LASER_PWM_IDLE_DUTY);
      }

      beam_detect_latch_reset(self);
      toggle_charge_pump(self); /* pulse once to prime charge pump before cut start */
      hrtimer_start(&self->charge_pump_timer, charge_pump_interval_ktime, HRTIMER_MODE_REL);
      /* Enable timer events */
      sdma_event_enable(self->sdmac, epit_sdma_event(self->epit));

      /* Start generating periodic events */
      /* If not accelerating, start at full speed */
      if (!opts.accelerate) {
        epit_start_hz(self->epit, self->step_freq);
      } else {
        epit_start_hz(self->epit, RAMP_MIN_STEP_FREQUENCY);
        _cnc_accel_start(self);
      }

      /* Set a nonzero priority to start the script */
      sdma_set_channel_priority(self->sdmac, 6);
      dev_dbg(self->dev, "started.");
    }
    spin_unlock_bh(&self->status_lock);
  }

  return ret;
}


int cnc_run(struct cnc *self)
{
  /* Run normally; no acceleration, deceleration, or waypoint interrupt */
  return cnc_run_with_options(self, (struct cnc_run_options){
    .num_steps = 0,
    .backward = false,
    .accelerate = false,
    .decelerate = false,
    .preserve_power = false
  });
}


int cnc_backtrack(struct cnc *self, uint32_t num_steps)
{
  if (num_steps == 0) {
    return -EINVAL;
  }
  /* Run backward, with acceleration, deceleration, and waypoint interrupt. */
  /* (Waypoint interrupt starts deceleration after num_steps.) */
  return cnc_run_with_options(self, (struct cnc_run_options){
    .num_steps = num_steps,
    .backward = true,
    .accelerate = true,
    .decelerate = true,
    .preserve_power = true
  });

}


int cnc_resume(struct cnc *self, uint32_t laser_delay_steps)
{
  /* Run forward, with acceleration and waypoint interrupt. */
  /* (Waypoint interrupt enables laser after laser_delay_steps.) */
  return cnc_run_with_options(self, (struct cnc_run_options){
    .num_steps = laser_delay_steps,
    .backward = false,
    .accelerate = true,
    .decelerate = false,
    .preserve_power = true
  });
}


/**
 * Idle: do nothing
 * Running: stop cut
 * Disabled: do nothing (remain in disabled state)
 * Fault: do nothing (return error)
 */
int cnc_stop(struct cnc *self)
{
  int ret = 0;
  spin_lock_bh(&self->status_lock);

  switch (self->status.state) {
    case STATE_IDLE:
    case STATE_DISABLED:
      break;

    case STATE_FAULT:
    default:
      ret = -EPERM;
      break;

    case STATE_RUNNING:
      /* Start a controlled deceleration. */
      _cnc_decel_start(self);
      break;
  }

  spin_unlock_bh(&self->status_lock);
  return ret;
}


/**
 * Idle: power off steppers
 * Running: stop everything and power off steppers
 * Disabled: do nothing
 * Fault: do nothing
 */
int cnc_disable(struct cnc *self)
{
  int ret = 0;
  spin_lock_bh(&self->status_lock);
  switch (self->status.state) {
    case STATE_IDLE:
    case STATE_RUNNING:
      _driver_stop(self, STATE_DISABLED);
      break;
    case STATE_DISABLED:
    case STATE_FAULT:
      break;
    default:
      ret = -EPERM;
      break;
  }
  spin_unlock_bh(&self->status_lock);
  return ret;
}


/**
 * Idle: do nothing
 * Running: error
 * Disabled: enable steppers
 * Fault: error (TODO)
 */
int cnc_enable(struct cnc *self)
{
  int ret = 0;
  spin_lock_bh(&self->status_lock);
  switch (self->status.state) {
    case STATE_DISABLED:
      if (_stepper_power_on(self, self->status.triggered_faults) == 0) {
        self->status.state = STATE_IDLE;
        cnc_notify_state_changed(self);
      } else {
        ret = -EPERM;
      }
      break;
    case STATE_IDLE:
      break;
    default:
      ret = -EPERM;
      break;
  }
  spin_unlock_bh(&self->status_lock);
  return ret;
}


/**
 * Idle: take step
 * Running: error
 * Disabled: take step (Z axis enable isn't controlled by kernel module)
 * Fault: error (TODO)
 */
int cnc_single_z_step(struct cnc *self, bool direction)
{
  int ret = 0;
  int z_step_gpio, z_dir_gpio;
  spin_lock_bh(&self->status_lock);
  switch (self->status.state) {
    case STATE_RUNNING:
    case STATE_FAULT:
      ret = -EPERM;
      break;
    default:
      ret = 0;
      break;
  }
  spin_unlock_bh(&self->status_lock);

  if (ret) {
    return ret;
  }

  z_step_gpio = self->gpios[PIN_Z_STEP];
  z_dir_gpio = self->gpios[PIN_Z_DIR];

  gpio_set_value(z_dir_gpio, direction);
  gpio_set_value(z_step_gpio, 1);
  udelay(2); /* DRV8818 wants a minimum 1us pulse duration */
  gpio_set_value(z_step_gpio, 0);
  gpio_set_value(z_dir_gpio, 0);
  return 0;
}


int cnc_clear_pulse_data(struct cnc *self, enum cnc_lseek_options opts)
{
  int ret = 0;
  uint32_t clear_flags = 0;
  switch (opts) {
    case PULSEDEV_LSEEK_CLEAR_DATA_AND_POSITION:
      clear_flags = CNC_BUFFER_CLEAR_DATA|CNC_BUFFER_CLEAR_POSITION;
      break;
    case PULSEDEV_LSEEK_CLEAR_DATA:
      clear_flags = CNC_BUFFER_CLEAR_DATA;
      break;
    case PULSEDEV_LSEEK_CLEAR_POSITION:
      clear_flags = CNC_BUFFER_CLEAR_POSITION;
      break;
    default:
      return -EINVAL;
  }

  spin_lock_bh(&self->status_lock);
  switch (self->status.state) {
    case STATE_RUNNING:
      ret = -EPERM;
      break;
    default:
      break;
  }
  spin_unlock_bh(&self->status_lock);

  /* don't touch the context if we're returning an error */
  if (ret) { return ret; }

  return cnc_buffer_clear(self, clear_flags);
}


int cnc_set_laser_latch(struct cnc *self, int value)
{
  /* If value == 0, latch is unlocked and LASER_ON is an output. */
  /* Otherwise, latch is locked and LASER_ON is high impedance. */
  gpio_set_value(self->gpios[PIN_LASER_LATCH_RESET], value);
  if (value == 0) {
    /* Allow LASER_ON to be driven by sdma. */
    gpio_direction_output(self->gpios[PIN_LASER_ON], 0);
  } else {
    gpio_direction_input(self->gpios[PIN_LASER_ON]);
  }
  return 0;
}


static void toggle_charge_pump(struct cnc *self)
{
  int gpio = self->gpios[PIN_CHARGE_PUMP];
  gpio_set_value(gpio, 0);
  gpio_set_value(gpio, 1);
  gpio_set_value(gpio, 0);
}


/* Called from hard interrupt context */
static enum hrtimer_restart charge_pump_timer_cb(struct hrtimer *timer)
{
  struct cnc *self = container_of(timer, struct cnc, charge_pump_timer);
  toggle_charge_pump(self);
  hrtimer_forward_now(timer, charge_pump_interval_ktime);
  return HRTIMER_RESTART;
}


enum cnc_state cnc_state(struct cnc *self)
{
  enum cnc_state ret;
  spin_lock_bh(&self->status_lock);
  ret = self->status.state;
  spin_unlock_bh(&self->status_lock);
  return ret;
}


#undef X
#define X(e,s) case e: return s;
const char *cnc_string_for_state(enum cnc_state st)
{
  switch (st) {
    DRIVER_STATES
    default: return "unknown";
  }
}


const char *cnc_state_string(struct cnc *self)
{
  return cnc_string_for_state(cnc_state(self));
}


u32 cnc_triggered_faults(struct cnc *self)
{
  u32 ret;
  spin_lock_bh(&self->status_lock);
  ret = self->status.triggered_faults;
  spin_unlock_bh(&self->status_lock);
  return ret;
}


ssize_t cnc_print_sdma_context(struct cnc *self, char *buf)
{
  return sdma_print_context(self->sdma, self->sdma_ch_num, buf);
}


int cnc_set_microstep_mode(struct cnc *self, enum cnc_axis axis, enum cnc_microstep_mode mode)
{
  int mode_binary, pin_mode0, pin_mode1, pin_mode2;
  switch (mode) {
    case MODE_FULL_STEP:     mode_binary = 0b000; break;
    case MODE_MICROSTEPS_2:  mode_binary = 0b001; break;
    case MODE_MICROSTEPS_4:  mode_binary = 0b010; break;
    case MODE_MICROSTEPS_8:  mode_binary = 0b011; break;
    case MODE_MICROSTEPS_16: mode_binary = 0b100; break;
    case MODE_MICROSTEPS_32: mode_binary = 0b101; break;
    default:                 return -EINVAL;
  }
  switch (axis) {
    case AXIS_X: pin_mode0 = PIN_X_MODE0; pin_mode1 = PIN_X_MODE1; pin_mode2 = PIN_X_MODE2; break;
    case AXIS_Y: pin_mode0 = PIN_Y_MODE0; pin_mode1 = PIN_Y_MODE1; pin_mode2 = PIN_Y_MODE2; break;
    default:     return -EINVAL;
  }
  gpio_set_value(self->gpios[pin_mode0], mode_binary & 0b001);
  gpio_set_value(self->gpios[pin_mode1], mode_binary & 0b010);
  gpio_set_value(self->gpios[pin_mode2], mode_binary & 0b100);
  return 0;
}


enum cnc_microstep_mode cnc_get_microstep_mode(struct cnc *self, enum cnc_axis axis)
{
  int mode_binary, pin_mode0, pin_mode1, pin_mode2;
  switch (axis) {
    case AXIS_X: pin_mode0 = PIN_X_MODE0; pin_mode1 = PIN_X_MODE1; pin_mode2 = PIN_X_MODE2; break;
    case AXIS_Y: pin_mode0 = PIN_Y_MODE0; pin_mode1 = PIN_Y_MODE1; pin_mode2 = PIN_Y_MODE2; break;
    default:     return -EINVAL;
  }
  mode_binary = (gpio_get_value(self->gpios[pin_mode0]) != 0) |
                ((gpio_get_value(self->gpios[pin_mode1]) != 0) << 1) |
                ((gpio_get_value(self->gpios[pin_mode2]) != 0) << 2);
  switch ((mode_binary) & 0b111) {
    case 0b000: return MODE_FULL_STEP;     break;
    case 0b001: return MODE_MICROSTEPS_2;  break;
    case 0b010: return MODE_MICROSTEPS_4;  break;
    case 0b011: return MODE_MICROSTEPS_8;  break;
    case 0b100: return MODE_MICROSTEPS_16; break;
    default:    return MODE_MICROSTEPS_32; break;
  }
}



int cnc_set_decay_mode(struct cnc *self, enum cnc_axis axis, enum cnc_decay_mode mode)
{
  int pin;
  switch (axis) {
    case AXIS_X: pin = PIN_X_DECAY; break;
    case AXIS_Y: pin = PIN_Y_DECAY; break;
    default:     return -EINVAL;
  }
  switch (mode) {
    case MODE_DECAY_SLOW:  gpio_direction_output(self->gpios[pin], 0); break;
    case MODE_DECAY_MIXED: gpio_direction_input(self->gpios[pin]); break;
    case MODE_DECAY_FAST:  gpio_direction_output(self->gpios[pin], 1); break;
    default:               return -EINVAL;
  }
  return 0;
}


enum cnc_decay_mode cnc_get_decay_mode(struct cnc *self, enum cnc_axis axis)
{
  int pin;
  struct gpio_desc *desc;
  switch (axis) {
    case AXIS_X: pin = PIN_X_DECAY; break;
    case AXIS_Y: pin = PIN_Y_DECAY; break;
    default:     return -EINVAL;
  }
  /* adapted from gpio_direction_show(), drivers/gpio/gpiolib.c */
  desc = gpio_to_desc(self->gpios[pin]);
  if (!desc) {
    return -EIO;
  }
  gpiod_get_direction(desc);
  if (test_bit(FLAG_IS_OUT, &desc->flags) == 0) {
    return MODE_DECAY_MIXED;
  } else {
    return (gpiod_get_value(desc)) ? MODE_DECAY_FAST : MODE_DECAY_SLOW;
  }
}


int cnc_set_motor_lock(struct cnc *self, u32 motor_lock_bits)
{
  /* Convert argument to a GPIO port bitmask. */
  u32 lock_val = 0;
  if (motor_lock_bits & MOTOR_LOCK_X)  { lock_val |= (1 << PIN_FROM_GPIO(self->gpios[PIN_X_STEP])); }
  if (motor_lock_bits & MOTOR_LOCK_Y1) { lock_val |= (1 << PIN_FROM_GPIO(self->gpios[PIN_Y1_STEP])); }
  if (motor_lock_bits & MOTOR_LOCK_Y2) { lock_val |= (1 << PIN_FROM_GPIO(self->gpios[PIN_Y2_STEP])); }
  if (motor_lock_bits & MOTOR_LOCK_Z)  { lock_val |= (1 << PIN_FROM_GPIO(self->gpios[PIN_Z_STEP])); }
  return sdma_set_reg(self->sdmac, &lock_val, ca);
}


u32 cnc_get_motor_lock(struct cnc *self)
{
  u32 lock_val = 0, retval = 0;
  if (sdma_get_reg(self->sdmac, &lock_val, ca)) {
    return 0;
  }
  /* Extract bits from the GPIO port bitmask. */
  if (lock_val & (1 << PIN_FROM_GPIO(self->gpios[PIN_X_STEP])))  { retval |= MOTOR_LOCK_X; }
  if (lock_val & (1 << PIN_FROM_GPIO(self->gpios[PIN_Y1_STEP]))) { retval |= MOTOR_LOCK_Y1; }
  if (lock_val & (1 << PIN_FROM_GPIO(self->gpios[PIN_Y2_STEP]))) { retval |= MOTOR_LOCK_Y2; }
  if (lock_val & (1 << PIN_FROM_GPIO(self->gpios[PIN_Z_STEP])))  { retval |= MOTOR_LOCK_Z; }
  return retval;
}


static void fault_tasklet_fn(unsigned long data)
{
  /* We don't need to protect the status field in this function */
  /* (see note in ramp_update_tasklet_fn()) */
  struct cnc *self = (struct cnc *)data;
  bool need_to_halt = false;
  int bit;

  /* Process each pending fault condition. */
  /* If a fatal fault has occurred, stop the driver immediately. */
  /* If a non-fatal fault has occurred, and the driver is running, */
  /* attempt a controlled deceleration. */
  for (bit = 0; bit < NUM_FAULT_CONDITIONS; bit++) {
    /* test_and_clear_bit() is atomic */
    if (test_and_clear_bit(bit, &self->pending_faults)) {
      dev_err(self->dev, "fault %d", (1 << bit));
      self->status.triggered_faults |= (1 << bit);
      if (FAULT_IS_FATAL(bit)) {
        need_to_halt = true;
      }
    }
  }

  /* sanity check: don't stop if no faults have occurred */
  if (self->status.triggered_faults) {
    if (need_to_halt) {
      dev_err(self->dev, "critical fault occurred: emergency stop");
      _driver_stop(self, STATE_FAULT);
    } else if (self->status.state == STATE_RUNNING) {
      _cnc_decel_start(self);
    }
  }
}


static inline void cnc_assert_fault(struct cnc *self, int fault_num)
{
  /* set_bit() is atomic */
  set_bit(fault_num, &self->pending_faults);
  tasklet_hi_schedule(&self->fault_tasklet);
}


/**
 * Handles FAULT falling edges.
 * Lower 2 bits of dev_id encode the fault signal that was asserted.
 */
static irqreturn_t cnc_fault_irq_handler(int irq, void *dev_id)
{
  struct cnc *self = CNC_FROM_FAULT_DEV_ID(dev_id);
  u32 fault_num = SIGNAL_FROM_FAULT_DEV_ID(dev_id);
  int pin, gpio;

  if (fault_num >= NUM_STEPPER_FAULT_SIGNALS) {
    return IRQ_HANDLED;
  }

  /* De-glitch: only fault if the line is actually low */
  pin = stepper_fault_gpios[fault_num];
  gpio = self->gpios[pin];
  if (gpio_get_value(gpio) != 0) {
    return IRQ_HANDLED;
  }

  if ((self->ignored_faults & (1 << fault_num)) == 0) {
    dev_err(self->dev, "driver fault detected! %d", fault_num);
    cnc_assert_fault(self, fault_num);
  }
  return IRQ_HANDLED;
}


static int cnc_register_fault_irqs(struct cnc *self)
{
  int i;
  int fault_irqs[NUM_STEPPER_FAULT_SIGNALS];
  int initial_fault_state = 0;

  /* Read the initial fault states and look up the irq numbers */
  for (i = 0; i < NUM_STEPPER_FAULT_SIGNALS; i++) {
    int pin_id = stepper_fault_gpios[i];
    int gpio = self->gpios[pin_id];
    int irq = gpio_to_irq(gpio);
    if (irq < 0) {
      dev_err(self->dev, "gpio %d has no irq", gpio);
      return irq;
    }
    fault_irqs[i] = irq;
    /* Fault signals are active low */
    initial_fault_state |= ((gpio_get_value(self->gpios[pin_id]) == 0) << i);
  }

  /* Are we initially in a fault state? */
  spin_lock_bh(&self->status_lock);
  self->status.triggered_faults = initial_fault_state & (~self->ignored_faults);
  if (initial_fault_state) {
    self->status.state = STATE_FAULT;
  }
  spin_unlock_bh(&self->status_lock);

  /* Register interrupt handlers */
  for (i = 0; i < NUM_STEPPER_FAULT_SIGNALS; i++) {
    int pin_id = stepper_fault_gpios[i];
    int irq = fault_irqs[i];
    /* Encode the fault signal number in the lower 2 bits of the dev_id. */
    /* Only care about falling edges (fault conditions) for now. */
    int ret = devm_request_irq(self->dev,
      irq,
      cnc_fault_irq_handler,
      IRQF_TRIGGER_FALLING,
      pin_configs[pin_id].name,
      FAULT_DEV_ID_FROM_CNC_AND_SIGNAL(self, i));
    if (ret) {
      dev_err(self->dev, "devm_request_irq(%d) failed: %d", irq, ret);
      return ret;
    }
  }
  return 0;
}


static void beam_detect_latch_reset(struct cnc *self)
{
  /* Pulse BEAM_DET_LATCH_RST high then low. */
  gpio_set_value(self->gpios[PIN_BEAM_LATCH_RESET], 1);
  gpio_set_value(self->gpios[PIN_BEAM_LATCH_RESET], 0);
}


#if INSTALL_PANIC_HANDLER
static int cnc_panic_handler(struct notifier_block *nb, unsigned long action, void *data)
{
  /* stop timers and make the hardware safe if there's a panic */
  struct cnc *self = container_of(nb, struct cnc, panic_notifier);
  _driver_stop(self, STATE_DISABLED); /* in atomic context; can't use cnc_disable() */
  dms_notifier_call_chain(&dms_notifier_list, 0, NULL); /* shut down other subsystems */
  return 0;
}
#endif


int cnc_probe(struct platform_device *pdev)
{
  struct cnc *self;
  struct device_node *epit_np = NULL;
  u32 sdma_params[2];
  int ret = 0;
  if (!cnc_enabled) { dev_info(&pdev->dev, "%s: disabled, skipping", __func__); return 0; }
  dev_info(&pdev->dev, "%s: started", __func__);

  /* Allocate driver data */
  self = devm_kzalloc(&pdev->dev, sizeof(*self), GFP_KERNEL);
  if (!self) {
    return -ENOMEM;
  }
  self->dev = &pdev->dev;

  mutex_init(&self->lock);
  spin_lock_init(&self->status_lock);
  tasklet_init(&self->fault_tasklet, fault_tasklet_fn, (unsigned long)self);
  cnc_set_step_frequency(self, STEP_FREQUENCY_DEFAULT);
  tasklet_hrtimer_init(&self->ramp_timer, ramp_update_tasklet_fn, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  hrtimer_init(&self->charge_pump_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  self->charge_pump_timer.function = charge_pump_timer_cb;

#if INITIAL_STATE_DISABLED
  self->status.state = STATE_DISABLED;
#else
  self->status.state = STATE_IDLE;
#endif

  /* Reserve memory for pulse data */
  ret = cnc_buffer_init(self);
  if (ret) {
    goto failed_buffer_init;
  }

  /* Set up GPIOs */
  ret = io_init_gpios(pdev->dev.of_node, pin_configs, self->gpios, NUM_GPIO_PINS);
  if (ret) {
    goto failed_io_init;
  }

  /* Set up PWM */
  ret = io_init_pwms(pdev->dev.of_node, &laser_pwm_config, &self->laser_pwm, 1);
  if (ret) {
    goto failed_pwm_init;
  }
  io_pwm_set_duty_cycle(&self->laser_pwm, LASER_PWM_IDLE_DUTY);

  /* Set up timer */
  epit_np = of_parse_phandle(pdev->dev.of_node, "timer", 0);
  if (IS_ERR(epit_np)) {
    dev_err(&pdev->dev, "no timer specified");
    ret = -ENODEV;
    goto failed_epit_init;
  }
  self->epit = epit_get(epit_np);
  of_node_put(epit_np);
  if (!self->epit) {
    goto failed_epit_init;
  }
  ret = epit_init_freerunning(self->epit, NULL, NULL);
  if (ret) {
    goto failed_epit_init;
  }

  /* Read SDMA channel number and load address */
  if (of_property_read_u32_array(pdev->dev.of_node, "sdma-params",
    sdma_params, ARRAY_SIZE(sdma_params)) == 0) {
    self->sdma_ch_num = sdma_params[0];
    self->sdma_script_origin = sdma_params[1];
  } else {
    dev_err(&pdev->dev, "sdma-params property not specified");
    goto failed_sdma_init;
  }
  /* Set up SDMA and get a channel reference */
  self->sdma = sdma_engine_get();
  if (!self->sdma) {
    goto failed_sdma_init;
  }
  self->sdmac = sdma_get_channel(self->sdma, self->sdma_ch_num);
  if (!self->sdmac) {
    goto failed_sdma_init;
  }

  /* Load the SDMA script */
  ret = load_sdma_script(self);
  if (ret) {
    goto failed_load_sdma_script;
  }
  sdma_set_channel_interrupt_callback(self->sdmac, cnc_sdma_interrupt, self);

  platform_set_drvdata(pdev, self);

  /* Create /dev/glowforge */
  self->pulsedev.minor = MISC_DYNAMIC_MINOR;
  self->pulsedev.name = PULSE_DEVICE_NAME;
  self->pulsedev.fops = &pulsedev_fops;
  self->pulsedev.parent = &pdev->dev;
  ret = misc_register(&self->pulsedev);
  if (ret) {
    dev_err(&pdev->dev, "unable to register " PULSE_DEVICE_PATH);
    goto failed_pulsedev_register;
  }

  /* Acquire the 40V supply */
  self->supply_40v = devm_regulator_get_exclusive(&pdev->dev, "40v");
  if (IS_ERR(self->supply_40v)) {
    dev_err(&pdev->dev, "failed to get 40V regulator");
    goto failed_regulator_get;
  }

  /* Register fault interrupt handlers */
  ret = cnc_register_fault_irqs(self);
  if (ret) {
    goto failed_register_fault_irqs;
  }

  /* Create sysfs attributes */
  ret = sysfs_create_group(&pdev->dev.kobj, &cnc_attr_group);
  if (ret < 0) {
    dev_err(&pdev->dev, "failed to register attribute group");
    goto failed_create_group;
  }
  self->state_attr_node = sysfs_get_dirent(pdev->dev.kobj.sd, STR(ATTR_STATE));
  if (!self->state_attr_node) {
    dev_err(&pdev->dev, "could not get node for state attribute");
    goto failed_create_link;
  }

  /* Add a link in /sys/glowforge */
  ret = sysfs_create_link(glowforge_kobj, &pdev->dev.kobj, CNC_GROUP_NAME);
  if (ret) {
    goto failed_create_link;
  }

#if !INITIAL_STATE_DISABLED
  /* Enable the steppers. */
  stepper_power_on(self);
#endif

#if INSTALL_PANIC_HANDLER
  /* Register panic handler */
  self->panic_notifier.notifier_call = cnc_panic_handler;
  atomic_notifier_chain_register(&panic_notifier_list, &self->panic_notifier);
#endif

  dev_info(&pdev->dev, "%s: done", __func__);
  return 0;

  sysfs_remove_link(&pdev->dev.kobj, CNC_GROUP_NAME);
failed_create_link:
  sysfs_remove_group(&pdev->dev.kobj, &cnc_attr_group);
failed_create_group:
failed_register_fault_irqs:
failed_regulator_get:
  misc_deregister(&self->pulsedev);
failed_pulsedev_register:
failed_load_sdma_script:
failed_sdma_init:
  epit_stop(self->epit);
failed_epit_init:
  io_release_pwms(&self->laser_pwm, 1);
failed_pwm_init:
  io_release_gpios(self->gpios, NUM_GPIO_PINS);
failed_io_init:
  cnc_buffer_destroy(self);
failed_buffer_init:
  mutex_destroy(&self->lock);
  return ret;
}


int cnc_remove(struct platform_device *pdev)
{
  struct cnc *self = platform_get_drvdata(pdev);
  if (!cnc_enabled) { return 0; }
  dev_info(&pdev->dev, "%s: started", __func__);
  epit_stop(self->epit);
  sdma_set_channel_interrupt_callback(self->sdmac, NULL, NULL);
  tasklet_hrtimer_cancel(&self->ramp_timer);
  hrtimer_cancel(&self->charge_pump_timer);
#if INSTALL_PANIC_HANDLER
  atomic_notifier_chain_unregister(&panic_notifier_list, &self->panic_notifier);
#endif
  io_release_pwms(&self->laser_pwm, 1);
  stepper_power_off(self);
  io_release_gpios(self->gpios, NUM_GPIO_PINS);
  self->state_attr_node = NULL;
  sysfs_remove_link(&pdev->dev.kobj, CNC_GROUP_NAME);
  sysfs_remove_group(&pdev->dev.kobj, &cnc_attr_group);
  misc_deregister(&self->pulsedev);
  cnc_buffer_destroy(self);
  mutex_destroy(&self->lock);
  tasklet_kill(&self->fault_tasklet);
  flush_scheduled_work();
  dev_info(&pdev->dev, "%s: done", __func__);
  return 0;
}
