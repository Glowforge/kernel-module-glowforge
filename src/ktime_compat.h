#ifndef KERNEL_SRC_KTIME_COMPAT_H_
#define KERNEL_SRC_KTIME_COMPAT_H_

#include <linux/ktime.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,14,28)
#define NSEC_TO_KTIME(tv64_) {.tv64 = (tv64_) }
#else
#define NSEC_TO_KTIME(tv64_) (tv64_)
#endif

#endif

