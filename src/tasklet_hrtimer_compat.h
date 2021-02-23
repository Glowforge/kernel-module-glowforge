#ifndef KERNEL_SRC_TASKLET_HRTIMER_COMPAT_H_
#define KERNEL_SRC_TASKLET_HRTIMER_COMPAT_H_

#include <linux/interrupt.h>
#include <linux/version.h>

/* tasklet_hrtimer was removed and replaced with the HRTIMER_MODE_REL_SOFT */
/* mode for regular hrtimers. */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,14,28)
#define SOFTIRQ_HRTIMER               struct tasklet_hrtimer
#define SOFTIRQ_HRTIMER_INIT(a,b,c)   tasklet_hrtimer_init((a), (b), (c), HRTIMER_MODE_REL)
#define SOFTIRQ_HRTIMER_START(a,b)    tasklet_hrtimer_start((a), (b), HRTIMER_MODE_REL)
#define SOFTIRQ_HRTIMER_CANCEL        tasklet_hrtimer_cancel
#define TO_SOFTIRQ_HRTIMER(t)         container_of(t, struct tasklet_hrtimer, timer)
#else
#define SOFTIRQ_HRTIMER               struct hrtimer
#define SOFTIRQ_HRTIMER_INIT(a,b,c)   do { hrtimer_init((a), (c), HRTIMER_MODE_REL_SOFT); (a)->function = (b); } while (0)
#define SOFTIRQ_HRTIMER_START(a,b)    hrtimer_start((a), (b), HRTIMER_MODE_REL_SOFT)
#define SOFTIRQ_HRTIMER_CANCEL        hrtimer_cancel
#define TO_SOFTIRQ_HRTIMER(t)         t
#endif

#endif

