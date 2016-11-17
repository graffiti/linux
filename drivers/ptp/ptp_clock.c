/*
 * PTP 1588 clock support
 *
 * Copyright (C) 2010 OMICRON electronics GmbH
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/idr.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/posix-clock.h>
#include <linux/pps_kernel.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>

#include "ptp_private.h"

#define PTP_MAX_ALARMS 4
#define PTP_PPS_DEFAULTS (PPS_CAPTUREASSERT | PPS_OFFSETASSERT)
#define PTP_PPS_EVENT PPS_CAPTUREASSERT
#define PTP_PPS_MODE (PTP_PPS_DEFAULTS | PPS_CANWAIT | PPS_TSFMT_TSPEC)
#define PTP_TIMER_MINIMUM_INTERVAL_NS 100000

/* private globals */

static dev_t ptp_devt;
static struct class *ptp_class;

static DEFINE_IDA(ptp_clocks_map);

/* time stamp event queue operations */

static inline int queue_free(struct timestamp_event_queue *q)
{
	return PTP_MAX_TIMESTAMPS - queue_cnt(q) - 1;
}

static void enqueue_external_timestamp(struct timestamp_event_queue *queue,
				       struct ptp_clock_event *src)
{
	struct ptp_extts_event *dst;
	unsigned long flags;
	s64 seconds;
	u32 remainder;

	seconds = div_u64_rem(src->timestamp, 1000000000, &remainder);

	spin_lock_irqsave(&queue->lock, flags);

	dst = &queue->buf[queue->tail];
	dst->index = src->index;
	dst->t.sec = seconds;
	dst->t.nsec = remainder;

	if (!queue_free(queue))
		queue->head = (queue->head + 1) % PTP_MAX_TIMESTAMPS;

	queue->tail = (queue->tail + 1) % PTP_MAX_TIMESTAMPS;

	spin_unlock_irqrestore(&queue->lock, flags);
}

static s32 scaled_ppm_to_ppb(long ppm)
{
	/*
	 * The 'freq' field in the 'struct timex' is in parts per
	 * million, but with a 16 bit binary fractional field.
	 *
	 * We want to calculate
	 *
	 *    ppb = scaled_ppm * 1000 / 2^16
	 *
	 * which simplifies to
	 *
	 *    ppb = scaled_ppm * 125 / 2^13
	 */
	s64 ppb = 1 + ppm;
	ppb *= 125;
	ppb >>= 13;
	return (s32) ppb;
}

/* posix clock implementation */

static int ptp_clock_getres(struct posix_clock *pc, struct timespec *tp)
{
	tp->tv_sec = 0;
	tp->tv_nsec = 1;
	return 0;
}

static int ptp_clock_settime(struct posix_clock *pc, const struct timespec *tp)
{
	struct ptp_clock *ptp = container_of(pc, struct ptp_clock, clock);
	struct timespec64 ts = timespec_to_timespec64(*tp);

	return  ptp->info->settime64(ptp->info, &ts);
}

static int ptp_clock_gettime(struct posix_clock *pc, struct timespec *tp)
{
	struct ptp_clock *ptp = container_of(pc, struct ptp_clock, clock);
	struct timespec64 ts;
	int err;

	err = ptp->info->gettime64(ptp->info, &ts);
	if (!err)
		*tp = timespec64_to_timespec(ts);
	return err;
}

static int ptp_clock_adjtime(struct posix_clock *pc, struct timex *tx)
{
	struct ptp_clock *ptp = container_of(pc, struct ptp_clock, clock);
	struct ptp_clock_info *ops;
	int err = -EOPNOTSUPP;

	ops = ptp->info;

	if (tx->modes & ADJ_SETOFFSET) {
		struct timespec ts;
		ktime_t kt;
		s64 delta;

		ts.tv_sec  = tx->time.tv_sec;
		ts.tv_nsec = tx->time.tv_usec;

		if (!(tx->modes & ADJ_NANO))
			ts.tv_nsec *= 1000;

		if ((unsigned long) ts.tv_nsec >= NSEC_PER_SEC)
			return -EINVAL;

		kt = timespec_to_ktime(ts);
		delta = ktime_to_ns(kt);
		err = ops->adjtime(ops, delta);
	} else if (tx->modes & ADJ_FREQUENCY) {
		s32 ppb = scaled_ppm_to_ppb(tx->freq);
		if (ppb > ops->max_adj || ppb < -ops->max_adj)
			return -ERANGE;
		err = ops->adjfreq(ops, ppb);
		ptp->dialed_frequency = tx->freq;
	} else if (tx->modes == 0) {
		tx->freq = ptp->dialed_frequency;
		err = 0;
	}

	return err;
}

static void alarm_event(struct ptp_clock *ptp, struct ptp_clock_event *event)
{
	struct timerqueue_node *next;
	struct k_itimer *kit;
	s64 ns_now;
	bool signal_failed_to_send;
	unsigned long tq_lock_flags;

	ns_now = timespec64_to_ns(&event->alarm_time);

	spin_lock_irqsave(&ptp->tq_lock, tq_lock_flags);

	next = timerqueue_getnext(&ptp->timerqueue);

	while (next) {
		if (next->expires.tv64 > ns_now)
			break;

		kit = container_of(next, struct k_itimer, it.real.timer.node);

		signal_failed_to_send = posix_timer_event(kit, 0);

		/* update the last one that has fired */
		timerqueue_del(&ptp->timerqueue, &kit->it.real.timer.node);
		if ((ktime_to_ns(kit->it.real.interval) != 0)
				&& !signal_failed_to_send) {
			/* this is a periodic timer, set the next fire time */
			kit->it.real.timer.node.expires =
					ktime_add(
						kit->it.real.timer.node.expires,
						  kit->it.real.interval);
			timerqueue_add(&ptp->timerqueue,
				       &kit->it.real.timer.node);
		}

		next = timerqueue_getnext(&ptp->timerqueue);
	}

	/* now set the event time to the next timer fire time */
	next = timerqueue_getnext(&ptp->timerqueue);
	if (next) {
		event->alarm_time = ktime_to_timespec64(next->expires);
	} else {
		event->alarm_time.tv_sec = 0;
		event->alarm_time.tv_nsec = 0;
	}

	spin_unlock_irqrestore(&ptp->tq_lock, tq_lock_flags);
}

static int ptp_timer_create(struct posix_clock *pc, struct k_itimer *kit)
{
	struct ptp_clock *ptp = container_of(pc, struct ptp_clock, clock);
	int err = 0;
	unsigned long tq_lock_flags;
	struct ptp_clock_request rq;

	if (ptp->info->enable == 0)
		return -EOPNOTSUPP;

	spin_lock_irqsave(&ptp->tq_lock, tq_lock_flags);

	if (ptp->number_of_timers == 0) {
		/* hardware timer is disabled, enable it */
		rq.type = PTP_CLK_REQ_ALARM;
		err = ptp->info->enable(ptp->info, &rq, 1);
	}

	if (err == 0) {
		timerqueue_init(&kit->it.real.timer.node);
		ptp->number_of_timers++;
	}

	spin_unlock_irqrestore(&ptp->tq_lock, tq_lock_flags);

	return err;
}

static int ptp_timer_delete(struct posix_clock *pc, struct k_itimer *kit)
{
	struct ptp_clock *ptp = container_of(pc, struct ptp_clock, clock);
	int err = 0;
	unsigned long tq_lock_flags;
	struct ptp_clock_request rq;

	if (ptp->info->enable == 0)
		return -EOPNOTSUPP;

	spin_lock_irqsave(&ptp->tq_lock, tq_lock_flags);

	if (!RB_EMPTY_NODE(&kit->it.real.timer.node.node))
		timerqueue_del(&ptp->timerqueue, &kit->it.real.timer.node);

	ptp->number_of_timers--;

	if (ptp->number_of_timers == 0) {
		/* there are no more timers set on this device,
		 * so we can disable the hardware timer
		 */
		rq.type = PTP_CLK_REQ_ALARM;
		err = ptp->info->enable(ptp->info, &rq, 0);
	}

	spin_unlock_irqrestore(&ptp->tq_lock, tq_lock_flags);

	return err;
}

static void ptp_timer_gettime(struct posix_clock *pc,
			      struct k_itimer *kit,
			      struct itimerspec *tsp)
{
	struct timespec time_now;

	if (ptp_clock_gettime(pc, &time_now) != 0)
		return;

	tsp->it_interval = ktime_to_timespec(kit->it.real.interval);
	tsp->it_value = timespec_sub(ktime_to_timespec(
			kit->it.real.timer.node.expires), time_now);
}


static int ptp_timer_settime(struct posix_clock *pc,
			     struct k_itimer *kit, int flags,
			     struct itimerspec *tsp, struct itimerspec *old)
{
	struct ptp_clock *ptp = container_of(pc, struct ptp_clock, clock);
	int err;
	unsigned long tq_lock_flags;
	struct timespec time_now;
	ktime_t fire_time;
	struct timerqueue_node *next;
	struct timespec64 ts;

	if (ptp->info->timersettime == 0)
		return -EOPNOTSUPP;

	if (old)
		ptp_timer_gettime(pc, kit, old);

	fire_time = timespec_to_ktime(tsp->it_value);

	if ((fire_time.tv64 != 0) && !(flags & TIMER_ABSTIME)) {
		err = ptp_clock_gettime(pc, &time_now);
		if (err)
			return err;
		/* convert relative to absolute time */
		fire_time = ktime_add(fire_time, timespec_to_ktime(time_now));
	}

	kit->it.real.interval = timespec_to_ktime(tsp->it_interval);

	if ((ktime_to_ns(kit->it.real.interval) != 0)
		&& (ktime_to_ns(kit->it.real.interval) < PTP_TIMER_MINIMUM_INTERVAL_NS))
		kit->it.real.interval = ns_to_ktime(PTP_TIMER_MINIMUM_INTERVAL_NS);

	spin_lock_irqsave(&ptp->tq_lock, tq_lock_flags);

	kit->it.real.timer.node.expires = fire_time;

	if (!RB_EMPTY_NODE(&kit->it.real.timer.node.node))
		timerqueue_del(&ptp->timerqueue, &kit->it.real.timer.node);

	if (fire_time.tv64 != 0)
		timerqueue_add(&ptp->timerqueue, &kit->it.real.timer.node);

	next = timerqueue_getnext(&ptp->timerqueue);

	spin_unlock_irqrestore(&ptp->tq_lock, tq_lock_flags);

	if (next)
		ts = ktime_to_timespec64(next->expires);
	else {
		ts.tv_sec = 0;
		ts.tv_nsec = 0;
	}
	return ptp->info->timersettime(ptp->info, &ts);
}

static struct posix_clock_operations ptp_clock_ops = {
	.owner		= THIS_MODULE,
	.clock_adjtime	= ptp_clock_adjtime,
	.clock_gettime	= ptp_clock_gettime,
	.clock_getres	= ptp_clock_getres,
	.clock_settime	= ptp_clock_settime,
	.timer_create	= ptp_timer_create,
	.timer_delete	= ptp_timer_delete,
	.timer_gettime	= ptp_timer_gettime,
	.timer_settime	= ptp_timer_settime,
	.ioctl		= ptp_ioctl,
	.open		= ptp_open,
	.poll		= ptp_poll,
	.read		= ptp_read,
};

static void delete_ptp_clock(struct posix_clock *pc)
{
	struct ptp_clock *ptp = container_of(pc, struct ptp_clock, clock);

	mutex_destroy(&ptp->tsevq_mux);
	mutex_destroy(&ptp->pincfg_mux);
	ida_simple_remove(&ptp_clocks_map, ptp->index);
	kfree(ptp);
}

/* public interface */

struct ptp_clock *ptp_clock_register(struct ptp_clock_info *info,
				     struct device *parent)
{
	struct ptp_clock *ptp;
	int err = 0, index, major = MAJOR(ptp_devt);

	if (info->n_alarm > PTP_MAX_ALARMS)
		return ERR_PTR(-EINVAL);

	/* Initialize a clock structure. */
	err = -ENOMEM;
	ptp = kzalloc(sizeof(struct ptp_clock), GFP_KERNEL);
	if (ptp == NULL)
		goto no_memory;

	index = ida_simple_get(&ptp_clocks_map, 0, MINORMASK + 1, GFP_KERNEL);
	if (index < 0) {
		err = index;
		goto no_slot;
	}

	ptp->clock.ops = ptp_clock_ops;
	ptp->clock.release = delete_ptp_clock;
	ptp->info = info;
	ptp->devid = MKDEV(major, index);
	ptp->index = index;
	spin_lock_init(&ptp->tsevq.lock);
	mutex_init(&ptp->tsevq_mux);
	mutex_init(&ptp->pincfg_mux);
	init_waitqueue_head(&ptp->tsev_wq);
	spin_lock_init(&ptp->tq_lock);
	timerqueue_init_head(&ptp->timerqueue);

	/* Create a new device in our class. */
	ptp->dev = device_create(ptp_class, parent, ptp->devid, ptp,
				 "ptp%d", ptp->index);
	if (IS_ERR(ptp->dev))
		goto no_device;

	dev_set_drvdata(ptp->dev, ptp);

	err = ptp_populate_sysfs(ptp);
	if (err)
		goto no_sysfs;

	/* Register a new PPS source. */
	if (info->pps) {
		struct pps_source_info pps;
		memset(&pps, 0, sizeof(pps));
		snprintf(pps.name, PPS_MAX_NAME_LEN, "ptp%d", index);
		pps.mode = PTP_PPS_MODE;
		pps.owner = info->owner;
		ptp->pps_source = pps_register_source(&pps, PTP_PPS_DEFAULTS);
		if (!ptp->pps_source) {
			pr_err("failed to register pps source\n");
			goto no_pps;
		}
	}

	/* Create a posix clock. */
	err = posix_clock_register(&ptp->clock, ptp->devid);
	if (err) {
		pr_err("failed to create posix clock\n");
		goto no_clock;
	}

	return ptp;

no_clock:
	if (ptp->pps_source)
		pps_unregister_source(ptp->pps_source);
no_pps:
	ptp_cleanup_sysfs(ptp);
no_sysfs:
	device_destroy(ptp_class, ptp->devid);
no_device:
	mutex_destroy(&ptp->tsevq_mux);
	mutex_destroy(&ptp->pincfg_mux);
no_slot:
	kfree(ptp);
no_memory:
	return ERR_PTR(err);
}
EXPORT_SYMBOL(ptp_clock_register);

int ptp_clock_unregister(struct ptp_clock *ptp)
{
	ptp->defunct = 1;
	wake_up_interruptible(&ptp->tsev_wq);

	/* Release the clock's resources. */
	if (ptp->pps_source)
		pps_unregister_source(ptp->pps_source);
	ptp_cleanup_sysfs(ptp);
	device_destroy(ptp_class, ptp->devid);

	posix_clock_unregister(&ptp->clock);
	return 0;
}
EXPORT_SYMBOL(ptp_clock_unregister);

void ptp_clock_event(struct ptp_clock *ptp, struct ptp_clock_event *event)
{
	struct pps_event_time evt;

	switch (event->type) {

	case PTP_CLOCK_ALARM:
		alarm_event(ptp, event);
		break;

	case PTP_CLOCK_EXTTS:
		enqueue_external_timestamp(&ptp->tsevq, event);
		wake_up_interruptible(&ptp->tsev_wq);
		break;

	case PTP_CLOCK_PPS:
		pps_get_ts(&evt);
		pps_event(ptp->pps_source, &evt, PTP_PPS_EVENT, NULL);
		break;

	case PTP_CLOCK_PPSUSR:
		pps_event(ptp->pps_source, &event->pps_times,
			  PTP_PPS_EVENT, NULL);
		break;
	}
}
EXPORT_SYMBOL(ptp_clock_event);

int ptp_clock_index(struct ptp_clock *ptp)
{
	return ptp->index;
}
EXPORT_SYMBOL(ptp_clock_index);

int ptp_find_pin(struct ptp_clock *ptp,
		 enum ptp_pin_function func, unsigned int chan)
{
	struct ptp_pin_desc *pin = NULL;
	int i;

	mutex_lock(&ptp->pincfg_mux);
	for (i = 0; i < ptp->info->n_pins; i++) {
		if (ptp->info->pin_config[i].func == func &&
		    ptp->info->pin_config[i].chan == chan) {
			pin = &ptp->info->pin_config[i];
			break;
		}
	}
	mutex_unlock(&ptp->pincfg_mux);

	return pin ? i : -1;
}
EXPORT_SYMBOL(ptp_find_pin);

/* module operations */

static void __exit ptp_exit(void)
{
	class_destroy(ptp_class);
	unregister_chrdev_region(ptp_devt, MINORMASK + 1);
	ida_destroy(&ptp_clocks_map);
}

static int __init ptp_init(void)
{
	int err;

	ptp_class = class_create(THIS_MODULE, "ptp");
	if (IS_ERR(ptp_class)) {
		pr_err("ptp: failed to allocate class\n");
		return PTR_ERR(ptp_class);
	}

	err = alloc_chrdev_region(&ptp_devt, 0, MINORMASK + 1, "ptp");
	if (err < 0) {
		pr_err("ptp: failed to allocate device region\n");
		goto no_region;
	}

	ptp_class->dev_groups = ptp_groups;
	pr_info("PTP clock support registered\n");
	return 0;

no_region:
	class_destroy(ptp_class);
	return err;
}

subsys_initcall(ptp_init);
module_exit(ptp_exit);

MODULE_AUTHOR("Richard Cochran <richardcochran@gmail.com>");
MODULE_DESCRIPTION("PTP clocks support");
MODULE_LICENSE("GPL");
