/**
 * device_attr.h
 *
 * Macro that simplifies creation of a sysfs attribute whose name is a macro.
 * (DEVICE_ATTR() does not work in this case.)
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

#ifndef KERNEL_SRC_DEVICE_ATTR_H_
#define KERNEL_SRC_DEVICE_ATTR_H_

#include <linux/compiler.h> /* for ___PASTE() */

/**
 * Wraps the DEVICE_ATTR() macro, forcing evaluation of each argument.
 * Must be used when the attribute name is a defined macro instead of a raw
 * identifier.
 */
#define DEFINE_DEVICE_ATTR(n,m,s,t) static DEVICE_ATTR(n,m,s,t)

/**
 * Forms a pointer to a device attribute whose name is a defined macro.
 * Example: if we have
 *   #define ATTR_FOO foo
 * then DEV_ATTR_PTR(ATTR_FOO) evaluates to
 *   &dev_attr_foo.attr
 */
#define DEV_ATTR_PTR(name)          ___PASTE(&dev_attr_, name) .attr

#endif
