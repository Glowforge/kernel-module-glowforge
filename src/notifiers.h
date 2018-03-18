/**
 * notifiers.h
 *
 * Module-wide notifier chains for fault handling.
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

#ifndef KERNEL_SRC_NOTIFIERS_H_
#define KERNEL_SRC_NOTIFIERS_H_

#include <linux/notifier.h>

/**
 * Subsystems may register callbacks with this notifier list to be called if the
 * "deadman switch" is tripped, i.e. a userspace process controlling the stepper
 * driver has crashed.
 * See cnc_api.c for more information on the deadman switch.
 */
extern struct atomic_notifier_head dms_notifier_list;

#define dms_notifier_chain_register   atomic_notifier_chain_register
#define dms_notifier_chain_unregister atomic_notifier_chain_unregister
#define dms_notifier_call_chain       atomic_notifier_call_chain

#endif
