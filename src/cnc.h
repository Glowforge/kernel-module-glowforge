/**
 * cnc.h
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
 * The step, direction, and laser enable pins are controlled by an SDMA script;
 * this subsystem is responsible for overseeing the operation of
 * the SDMA engine, the periodic interval timer (EPIT) that drives it,
 * and the GPIO port settings.
 */

#ifndef KERNEL_SRC_CNC_H_
#define KERNEL_SRC_CNC_H_

#include <linux/platform_device.h>

int cnc_probe(struct platform_device *pdev);

int cnc_remove(struct platform_device *pdev);

#endif
