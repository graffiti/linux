/*
 * Fast Ethernet Controller (ENET) PTP driver for MX6x.
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/fec.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_net.h>

#include "fec.h"

/* FEC 1588 register bits */
#define FEC_T_CTRL_SLAVE                0x00002000
#define FEC_T_CTRL_CAPTURE              0x00000800
#define FEC_T_CTRL_RESTART              0x00000200
#define FEC_T_CTRL_PERIOD_RST           0x00000030
#define FEC_T_CTRL_PERIOD_EN		0x00000010
#define FEC_T_CTRL_ENABLE               0x00000001

#define FEC_T_INC_MASK                  0x0000007f
#define FEC_T_INC_OFFSET                0
#define FEC_T_INC_CORR_MASK             0x00007f00
#define FEC_T_INC_CORR_OFFSET           8

#define FEC_T_CTRL_PINPER		0x00000080
#define FEC_T_TF0_MASK			0x00000001
#define FEC_T_TF0_OFFSET		0
#define FEC_T_TF1_MASK			0x00000002
#define FEC_T_TF1_OFFSET		1
#define FEC_T_TF2_MASK			0x00000004
#define FEC_T_TF2_OFFSET		2
#define FEC_T_TF3_MASK			0x00000008
#define FEC_T_TF3_OFFSET		3
#define FEC_T_TDRE_MASK			0x00000001
#define FEC_T_TDRE_OFFSET		0
#define FEC_T_TMODE_MASK		0x0000003C
#define FEC_T_TMODE_OFFSET		2
#define FEC_T_TIE_MASK			0x00000040
#define FEC_T_TIE_OFFSET		6
#define FEC_T_TF_MASK			0x00000080
#define FEC_T_TF_OFFSET			7

#define FEC_ATIME_CTRL		0x400
#define FEC_ATIME		0x404
#define FEC_ATIME_EVT_OFFSET	0x408
#define FEC_ATIME_EVT_PERIOD	0x40c
#define FEC_ATIME_CORR		0x410
#define FEC_ATIME_INC		0x414
#define FEC_TS_TIMESTAMP	0x418

#define FEC_ATVR		FEC_ATIME
#define FEC_ATCR		FEC_ATIME_CTRL
#define FEC_ATINC		FEC_ATIME_INC
#define FEC_ATSTMP		FEC_TS_TIMESTAMP
#define FEC_ATPER		FEC_ATIME_EVT_PERIOD
#define FEC_ATOFF		FEC_ATIME_EVT_OFFSET
#define FEC_TGSR		0x604
#define FEC_TCSR(n)		(0x608 + n * 0x08)
#define FEC_TCCR(n)		(0x60C + n * 0x08)
#define MAX_TIMER_CHANNEL	3
#define FEC_TMODE_DISABLED	0x00
#define FEC_TMODE_SOFTWARE	0x04
#define FEC_TMODE_TOGGLE	0x05
#define FEC_HIGH_PULSE		0x0F

#define FEC_CC_MULT	(1 << 31)
#define FEC_COUNTER_PERIOD	(1 << 31)
#define PPS_OUPUT_RELOAD_PERIOD	NSEC_PER_SEC
#define FEC_CHANNLE_0		0
#define DEFAULT_PPS_CHANNEL	FEC_CHANNLE_0

static int fec_ptp_timer_enable(struct ptp_clock_info *ptp, bool enable);

/**
 * fec_ptp_enable_pps
 * @fep: the fec_enet_private structure handle
 * @enable: enable the channel pps output
 *
 * This function enables the PPS output on the timer channel.
 */
static int fec_ptp_enable_pps(struct fec_enet_private *fep, uint enable)
{
	unsigned long flags;
	u32 val, tempval;
	int inc;
	struct timespec ts;
	u64 ns;
	u32 remainder;
	val = 0;

	if (!(fep->hwts_tx_en || fep->hwts_rx_en)) {
		dev_err(&fep->pdev->dev, "No ptp stack is running\n");
		return -EINVAL;
	}

	if (fep->pps_enable == enable)
		return 0;

	fep->pps_channel = DEFAULT_PPS_CHANNEL;
	fep->reload_period = PPS_OUPUT_RELOAD_PERIOD;
	inc = fep->ptp_inc;

	spin_lock_irqsave(&fep->tmreg_lock, flags);

	if (enable) {
		/* clear capture or output compare interrupt status if have.
		 */
		writel(FEC_T_TF_MASK, fep->hwp + FEC_TCSR(fep->pps_channel));

		/* It is recommended to double check the TMODE field in the
		 * TCSR register to be cleared before the first compare counter
		 * is written into TCCR register. Just add a double check.
		 */
		val = readl(fep->hwp + FEC_TCSR(fep->pps_channel));
		do {
			val &= ~(FEC_T_TMODE_MASK);
			writel(val, fep->hwp + FEC_TCSR(fep->pps_channel));
			val = readl(fep->hwp + FEC_TCSR(fep->pps_channel));
		} while (val & FEC_T_TMODE_MASK);

		/* Dummy read counter to update the counter */
		timecounter_read(&fep->tc);
		/* We want to find the first compare event in the next
		 * second point. So we need to know what the ptp time
		 * is now and how many nanoseconds is ahead to get next second.
		 * The remaining nanosecond ahead before the next second would be
		 * NSEC_PER_SEC - ts.tv_nsec. Add the remaining nanoseconds
		 * to current timer would be next second.
		 */
		tempval = readl(fep->hwp + FEC_ATIME_CTRL);
		tempval |= FEC_T_CTRL_CAPTURE;
		writel(tempval, fep->hwp + FEC_ATIME_CTRL);

		tempval = readl(fep->hwp + FEC_ATIME);
		/* Convert the ptp local counter to 1588 timestamp */
		ns = timecounter_cyc2time(&fep->tc, tempval);
		ts.tv_sec = div_u64_rem(ns, 1000000000ULL, &remainder);
		ts.tv_nsec = remainder;

		/* The tempval is  less than 3 seconds, and  so val is less than
		 * 4 seconds. No overflow for 32bit calculation.
		 */
		val = NSEC_PER_SEC - (u32)ts.tv_nsec + tempval;

		/* Need to consider the situation that the current time is
		 * very close to the second point, which means NSEC_PER_SEC
		 * - ts.tv_nsec is close to be zero(For example 20ns); Since the timer
		 * is still running when we calculate the first compare event, it is
		 * possible that the remaining nanoseonds run out before the compare
		 * counter is calculated and written into TCCR register. To avoid
		 * this possibility, we will set the compare event to be the next
		 * of next second. The current setting is 31-bit timer and wrap
		 * around over 2 seconds. So it is okay to set the next of next
		 * seond for the timer.
		 */
		val += NSEC_PER_SEC;

		/* We add (2 * NSEC_PER_SEC - (u32)ts.tv_nsec) to current
		 * ptp counter, which maybe cause 32-bit wrap. Since the
		 * (NSEC_PER_SEC - (u32)ts.tv_nsec) is less than 2 second.
		 * We can ensure the wrap will not cause issue. If the offset
		 * is bigger than fep->cc.mask would be a error.
		 */
		val &= fep->cc.mask;
		writel(val, fep->hwp + FEC_TCCR(fep->pps_channel));

		/* Calculate the second the compare event timestamp */
		fep->next_counter = (val + fep->reload_period) & fep->cc.mask;

		/* * Enable compare event when overflow */
		val = readl(fep->hwp + FEC_ATIME_CTRL);
		val |= FEC_T_CTRL_PINPER;
		writel(val, fep->hwp + FEC_ATIME_CTRL);

		/* Compare channel setting. */
		val = readl(fep->hwp + FEC_TCSR(fep->pps_channel));
		val |= (1 << FEC_T_TF_OFFSET | 1 << FEC_T_TIE_OFFSET);
		val &= ~(1 << FEC_T_TDRE_OFFSET);
		val &= ~(FEC_T_TMODE_MASK);
		val |= (FEC_HIGH_PULSE << FEC_T_TMODE_OFFSET);
		writel(val, fep->hwp + FEC_TCSR(fep->pps_channel));

		/* Write the second compare event timestamp and calculate
		 * the third timestamp. Refer the TCCR register detail in the spec.
		 */
		writel(fep->next_counter, fep->hwp + FEC_TCCR(fep->pps_channel));
		fep->next_counter = (fep->next_counter + fep->reload_period) & fep->cc.mask;
	} else {
		writel(0, fep->hwp + FEC_TCSR(fep->pps_channel));
	}

	fep->pps_enable = enable;
	spin_unlock_irqrestore(&fep->tmreg_lock, flags);

	return 0;
}

/**
 * fec_ptp_read - read raw cycle counter (to be used by time counter)
 * @cc: the cyclecounter structure
 *
 * this function reads the cyclecounter registers and is called by the
 * cyclecounter structure used to construct a ns counter from the
 * arbitrary fixed point registers
 */
static cycle_t fec_ptp_read(const struct cyclecounter *cc)
{
	struct fec_enet_private *fep =
		container_of(cc, struct fec_enet_private, cc);
	const struct platform_device_id *id_entry =
		platform_get_device_id(fep->pdev);
	u32 tempval;

	tempval = readl(fep->hwp + FEC_ATIME_CTRL);
	tempval |= FEC_T_CTRL_CAPTURE;
	writel(tempval, fep->hwp + FEC_ATIME_CTRL);

	if (id_entry->driver_data & FEC_QUIRK_BUG_CAPTURE)
		udelay(1);

	return readl(fep->hwp + FEC_ATIME);
}

/**
 * fec_ptp_start_cyclecounter - create the cycle counter from hw
 * @ndev: network device
 *
 * this function initializes the timecounter and cyclecounter
 * structures for use in generated a ns counter from the arbitrary
 * fixed point cycles registers in the hardware.
 */
void fec_ptp_start_cyclecounter(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	unsigned long flags;
	int inc;

	inc = 1000000000 / fep->cycle_speed;

	/* grab the ptp lock */
	spin_lock_irqsave(&fep->tmreg_lock, flags);

	/* 1ns counter */
	writel(inc << FEC_T_INC_OFFSET, fep->hwp + FEC_ATIME_INC);

	/* use 31-bit timer counter */
	writel(FEC_COUNTER_PERIOD, fep->hwp + FEC_ATIME_EVT_PERIOD);

	writel(FEC_T_CTRL_ENABLE | FEC_T_CTRL_PERIOD_RST,
		fep->hwp + FEC_ATIME_CTRL);

	memset(&fep->cc, 0, sizeof(fep->cc));
	fep->cc.read = fec_ptp_read;
	fep->cc.mask = CLOCKSOURCE_MASK(31);
	fep->cc.shift = 31;
	fep->cc.mult = FEC_CC_MULT;

	/* reset the ns time counter */
	timecounter_init(&fep->tc, &fep->cc, ktime_to_ns(ktime_get_real()));

	spin_unlock_irqrestore(&fep->tmreg_lock, flags);
}

/**
 * fec_ptp_adjfreq - adjust ptp cycle frequency
 * @ptp: the ptp clock structure
 * @ppb: parts per billion adjustment from base
 *
 * Adjust the frequency of the ptp cycle counter by the
 * indicated ppb from the base frequency.
 *
 * Because ENET hardware frequency adjust is complex,
 * using software method to do that.
 */
static int fec_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	unsigned long flags;
	int neg_adj = 0;
	u32 i, tmp;
	u32 corr_inc, corr_period;
	u32 corr_ns;
	u64 lhs, rhs;

	struct fec_enet_private *fep =
	    container_of(ptp, struct fec_enet_private, ptp_caps);

	if (ppb == 0)
		return 0;

	if (ppb < 0) {
		ppb = -ppb;
		neg_adj = 1;
	}

	/* In theory, corr_inc/corr_period = ppb/NSEC_PER_SEC;
	 * Try to find the corr_inc  between 1 to fep->ptp_inc to
	 * meet adjustment requirement.
	 */
	lhs = NSEC_PER_SEC;
	rhs = (u64)ppb * (u64)fep->ptp_inc;
	for (i = 1; i <= fep->ptp_inc; i++) {
		if (lhs >= rhs) {
			corr_inc = i;
			corr_period = div_u64(lhs, rhs);
			break;
		}
		lhs += NSEC_PER_SEC;
	}
	/* Not found? Set it to high value - double speed
	 * correct in every clock step.
	 */
	if (i > fep->ptp_inc) {
		corr_inc = fep->ptp_inc;
		corr_period = 1;
	}

	if (neg_adj)
		corr_ns = fep->ptp_inc - corr_inc;
	else
		corr_ns = fep->ptp_inc + corr_inc;

	spin_lock_irqsave(&fep->tmreg_lock, flags);

	tmp = readl(fep->hwp + FEC_ATIME_INC) & FEC_T_INC_MASK;
	tmp |= corr_ns << FEC_T_INC_CORR_OFFSET;
	writel(tmp, fep->hwp + FEC_ATIME_INC);
	writel(corr_period, fep->hwp + FEC_ATIME_CORR);
	/* dummy read to update the timer. */
	timecounter_read(&fep->tc);

	spin_unlock_irqrestore(&fep->tmreg_lock, flags);

	return 0;
}

/**
 * fec_ptp_adjtime
 * @ptp: the ptp clock structure
 * @delta: offset to adjust the cycle counter by
 *
 * adjust the timer by resetting the timecounter structure.
 */
static int fec_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct fec_enet_private *fep =
	    container_of(ptp, struct fec_enet_private, ptp_caps);
	unsigned long flags;

	spin_lock_irqsave(&fep->tmreg_lock, flags);
	timecounter_adjtime(&fep->tc, delta);
	spin_unlock_irqrestore(&fep->tmreg_lock, flags);

	return 0;
}

/**
 * fec_ptp_gettime
 * @ptp: the ptp clock structure
 * @ts: timespec structure to hold the current time value
 *
 * read the timecounter and return the correct value on ns,
 * after converting it into a struct timespec.
 */
static int fec_ptp_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct fec_enet_private *adapter =
	    container_of(ptp, struct fec_enet_private, ptp_caps);
	u64 ns;
	unsigned long flags;

	spin_lock_irqsave(&adapter->tmreg_lock, flags);
	ns = timecounter_read(&adapter->tc);


	//kjt debug
	printk(KERN_ALERT "fec: ATCR = %x\n", readl(adapter->hwp + FEC_ATCR));
	printk(KERN_ALERT "fec: ATINC = %x\n", readl(adapter->hwp + FEC_ATINC));
	printk(KERN_ALERT "fec: ATSTMP = %x\n", readl(adapter->hwp + FEC_ATSTMP));
	printk(KERN_ALERT "fec: ATPER = %x\n", readl(adapter->hwp + FEC_ATPER));
	printk(KERN_ALERT "fec: ATOFF = %x\n", readl(adapter->hwp + FEC_ATOFF));
	printk(KERN_ALERT "fec: ENET_ECR = %x\n", readl(adapter->hwp + FEC_ECNTRL));

	writel((u32)1<<11, adapter->hwp + FEC_ATCR);
	printk(KERN_ALERT "fec: ATVR = %x\n", readl(adapter->hwp + FEC_ATVR));
	//end kjt debug



	spin_unlock_irqrestore(&adapter->tmreg_lock, flags);

	*ts = ns_to_timespec64(ns);

	return 0;
}

/**
 * fec_ptp_settime
 * @ptp: the ptp clock structure
 * @ts: the timespec containing the new time for the cycle counter
 *
 * reset the timecounter to use a new base value instead of the kernel
 * wall timer value.
 */
static int fec_ptp_settime(struct ptp_clock_info *ptp,
			   const struct timespec64 *ts)
{
	struct fec_enet_private *fep =
	    container_of(ptp, struct fec_enet_private, ptp_caps);

	u64 ns;
	unsigned long flags;
	u32 counter;

	mutex_lock(&fep->ptp_clk_mutex);
	/* Check the ptp clock */
	if (!fep->ptp_clk_on) {
		mutex_unlock(&fep->ptp_clk_mutex);
		return -EINVAL;
	}

	ns = timespec64_to_ns(ts);
	/* Get the timer value based on timestamp.
	 * Update the counter with the masked value.
	 */
	counter = ns & fep->cc.mask;

	spin_lock_irqsave(&fep->tmreg_lock, flags);
	writel(counter, fep->hwp + FEC_ATIME);
	timecounter_init(&fep->tc, &fep->cc, ns);
	spin_unlock_irqrestore(&fep->tmreg_lock, flags);
	mutex_unlock(&fep->ptp_clk_mutex);
	return 0;
}

/**
 * fec_ptp_enable
 * @ptp: the ptp clock structure
 * @rq: the requested feature to change
 * @on: whether to enable or disable the feature
 *
 */
static int fec_ptp_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	struct fec_enet_private *fep =
	    container_of(ptp, struct fec_enet_private, ptp_caps);
	int use_freq = 0, pin = -1;
	struct timespec64 ts;
	s64 ns;

	switch (rq->type)
	{
	case PTP_CLK_REQ_EXTTS:
		if (on) {
			pin = ptp_find_pin(fep->ptp_clock, PTP_PF_EXTTS,
					rq->extts.index);
			if (pin < 0)
				return -EBUSY;
		}
/*		if (rq->extts.index == 1) {
			tsauxc_mask = TSAUXC_EN_TS1;
			tsim_mask = TSINTR_AUTT1;
		} else {
			tsauxc_mask = TSAUXC_EN_TS0;
			tsim_mask = TSINTR_AUTT0;
		}
		spin_lock_irqsave(&igb->tmreg_lock, flags);
		tsauxc = rd32(E1000_TSAUXC);
		tsim = rd32(E1000_TSIM);
		if (on) {
			igb_pin_extts(igb, rq->extts.index, pin);
			tsauxc |= tsauxc_mask;
			tsim |= tsim_mask;
		} else {
			tsauxc &= ~tsauxc_mask;
			tsim &= ~tsim_mask;
		}
		wr32(E1000_TSAUXC, tsauxc);
		wr32(E1000_TSIM, tsim);
		spin_unlock_irqrestore(&igb->tmreg_lock, flags);
		*/
		return 0;

	case PTP_CLK_REQ_PEROUT:
		/* do we have a timer enabled on the requested channel? */
		if (fep->timer_enabled && (rq->perout.index == fep->sdp_config[FEC_SDP_TIMER].chan))
			return -EBUSY;
		if (on) {
			pin = ptp_find_pin(fep->ptp_clock, PTP_PF_PEROUT,
					rq->perout.index);
			if (pin < 0)
				return -EBUSY;
		}
		ts.tv_sec = rq->perout.period.sec;
		ts.tv_nsec = rq->perout.period.nsec;
		ns = timespec64_to_ns(&ts);
		ns = ns >> 1;
		if (on && ns <= 70000000LL) {
			if (ns < 8LL)
				return -EINVAL;
			use_freq = 1;
		}
		ts = ns_to_timespec64(ns);
/*		if (rq->perout.index == 1) {
			if (use_freq) {
				tsauxc_mask = TSAUXC_EN_CLK1 | TSAUXC_ST1;
				tsim_mask = 0;
			} else {
				tsauxc_mask = TSAUXC_EN_TT1;
				tsim_mask = TSINTR_TT1;
			}
			trgttiml = E1000_TRGTTIML1;
			trgttimh = E1000_TRGTTIMH1;
			freqout = E1000_FREQOUT1;
		} else {
			if (use_freq) {
				tsauxc_mask = TSAUXC_EN_CLK0 | TSAUXC_ST0;
				tsim_mask = 0;
			} else {
				tsauxc_mask = TSAUXC_EN_TT0;
				tsim_mask = TSINTR_TT0;
			}
			trgttiml = E1000_TRGTTIML0;
			trgttimh = E1000_TRGTTIMH0;
			freqout = E1000_FREQOUT0;
		}
		spin_lock_irqsave(&igb->tmreg_lock, flags);
		tsauxc = rd32(E1000_TSAUXC);
		tsim = rd32(E1000_TSIM);
		if (rq->perout.index == 1) {
			tsauxc &= ~(TSAUXC_EN_TT1 | TSAUXC_EN_CLK1 | TSAUXC_ST1);
			tsim &= ~TSINTR_TT1;
		} else {
			tsauxc &= ~(TSAUXC_EN_TT0 | TSAUXC_EN_CLK0 | TSAUXC_ST0);
			tsim &= ~TSINTR_TT0;
		}
		if (on) {
			int i = rq->perout.index;
			igb_pin_perout(igb, i, pin, use_freq);
			igb->perout[i].start.tv_sec = rq->perout.start.sec;
			igb->perout[i].start.tv_nsec = rq->perout.start.nsec;
			igb->perout[i].period.tv_sec = ts.tv_sec;
			igb->perout[i].period.tv_nsec = ts.tv_nsec;
			wr32(trgttimh, rq->perout.start.sec);
			wr32(trgttiml, rq->perout.start.nsec);
			if (use_freq)
				wr32(freqout, ns);
			tsauxc |= tsauxc_mask;
			tsim |= tsim_mask;
		}
		wr32(E1000_TSAUXC, tsauxc);
		wr32(E1000_TSIM, tsim);
		spin_unlock_irqrestore(&igb->tmreg_lock, flags);
		*/
		return 0;

	case PTP_CLK_REQ_ALARM:
		return fec_ptp_timer_enable(ptp, on!=0);

	case PTP_CLK_REQ_PPS:
		return fec_ptp_enable_pps(fep, on);
	}

	return -EOPNOTSUPP;





}

/**
 * fec_ptp_hwtstamp_ioctl - control hardware time stamping
 * @ndev: pointer to net_device
 * @ifreq: ioctl data
 * @cmd: particular ioctl requested
 */
int fec_ptp_set(struct net_device *ndev, struct ifreq *ifr)
{
	struct fec_enet_private *fep = netdev_priv(ndev);

	struct hwtstamp_config config;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		fep->hwts_tx_en = 0;
		break;
	case HWTSTAMP_TX_ON:
		fep->hwts_tx_en = 1;
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		if (fep->hwts_rx_en)
			fep->hwts_rx_en = 0;
		config.rx_filter = HWTSTAMP_FILTER_NONE;
		break;

	default:
		/*
		 * register RXMTRL must be set in order to do V1 packets,
		 * therefore it is not possible to time stamp both V1 Sync and
		 * Delay_Req messages and hardware does not support
		 * timestamping all packets => return error
		 */
		fep->hwts_rx_en = 1;
		config.rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	}

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
	    -EFAULT : 0;
}

int fec_ptp_get(struct net_device *ndev, struct ifreq *ifr)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	struct hwtstamp_config config;

	config.flags = 0;
	config.tx_type = fep->hwts_tx_en ? HWTSTAMP_TX_ON : HWTSTAMP_TX_OFF;
	config.rx_filter = (fep->hwts_rx_en ?
			    HWTSTAMP_FILTER_ALL : HWTSTAMP_FILTER_NONE);

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
}

/**
 * fec_time_keep - call timecounter_read every second to avoid timer overrun
 *                 because ENET just support 32bit counter, will timeout in 4s
 */
static void fec_time_keep(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct fec_enet_private *fep = container_of(dwork, struct fec_enet_private, time_keep);
	u64 ns;
	unsigned long flags;

	mutex_lock(&fep->ptp_clk_mutex);
	if (fep->ptp_clk_on) {
		spin_lock_irqsave(&fep->tmreg_lock, flags);
		ns = timecounter_read(&fep->tc);
		spin_unlock_irqrestore(&fep->tmreg_lock, flags);
	}
	mutex_unlock(&fep->ptp_clk_mutex);

	schedule_delayed_work(&fep->time_keep, HZ);
}

/* tm_reg lock must be held for this call */
bool fec_tc_enable(struct fec_enet_private *fep, bool enable, u64 ns)
{
	u32 val;
	u64 time_now, nsShifted;
	int tries = 0;
	int chan = fep->sdp_config[FEC_SDP_TIMER].chan;

	if(enable)
	{
		//first disable it
		fec_tc_enable(fep, false, 0);


		//set trigger time
		time_now = timecounter_read(&fep->tc);

		//printk(KERN_ALERT "fec_tc_enable: time now = %llu, tc->cycle_last = %llu, tc->nsec = %llu\n", time_now, fep->tc.cycle_last, fep->tc.nsec);

		nsShifted = ns - ((fep->tc.nsec & fep->cc.mask) - fep->tc.cycle_last);

		val = nsShifted & fep->cc.mask;
		writel(val, fep->hwp + FEC_TCCR(chan));

		/* Compare channel setting. */
		val = readl(fep->hwp + FEC_TCSR(chan));
		val |= (1 << FEC_T_TF_OFFSET);
		val |= (1 << FEC_T_TIE_OFFSET);
		val &= ~(1 << FEC_T_TDRE_OFFSET);
		val &= ~(FEC_T_TMODE_MASK);
		val |= (FEC_TMODE_SOFTWARE << FEC_T_TMODE_OFFSET);
		writel(val, fep->hwp + FEC_TCSR(chan));

		while( tries<100 && ((readl(fep->hwp + FEC_TCSR(chan)) & FEC_T_TMODE_MASK) != (FEC_TMODE_SOFTWARE << FEC_T_TMODE_OFFSET)) )
		{
			writel(val, fep->hwp + FEC_TCSR(chan));
			tries++;
		}
		if(tries == 100)
		{
			printk(KERN_ALERT "TSCR stuck on %d writing %d try %d\n", readl(fep->hwp + FEC_TCSR(chan)), val, tries);
		}

		if (ns!=0)
		{
			time_now = timecounter_read(&fep->tc);

			if(time_now>ns)
			{
				//the event already passed
				return false;
			}
		}
		return true;
	}
	else
	{
		//disable
		/* Compare channel setting. */
		val = readl(fep->hwp + FEC_TCSR(chan));
		val |= (1 << FEC_T_TF_OFFSET);
		val &= ~(1 << FEC_T_TDRE_OFFSET);
		val &= ~(1 << FEC_T_TIE_OFFSET);
		val &= ~(FEC_T_TMODE_MASK);
		writel(val, fep->hwp + FEC_TCSR(chan));

		while( (readl(fep->hwp + FEC_TCSR(chan)) & FEC_T_TMODE_MASK) != (FEC_TMODE_DISABLED << FEC_T_TMODE_OFFSET) )
		{
			printk(KERN_ALERT ".");
		}
		while (readl(fep->hwp + FEC_TCSR(chan)) & FEC_T_TF_MASK)
		{
			writel(val, fep->hwp + FEC_TCSR(chan));
			printk(KERN_ALERT "-");
		}
		return true;
	}
}

static int fec_ptp_timer_settime(struct ptp_clock_info *ptp,
				      struct timespec64 *ts)
{
	struct fec_enet_private *fep =
		    container_of(ptp, struct fec_enet_private, ptp_caps);
	s64 ns;
	unsigned long irqsaveflags;
	struct ptp_clock_event event;

	printk(KERN_ALERT "fec_ptp_timer_settime: ts %d:%d\n", (int)ts->tv_sec, (int)ts->tv_nsec);

	ns = timespec64_to_ns(ts);
	mutex_lock(&fep->ptp_clk_mutex);
	spin_lock_irqsave(&fep->tmreg_lock, irqsaveflags);

	/* disable timer */
	fec_tc_enable(fep, false, 0);

	/* set trigger time and enable timer */
	while ((ns !=0) && !fec_tc_enable(fep, true, ns))
	{
		event.type = PTP_CLOCK_ALARM;
		event.index = 0;
		/* ptp_clock_event will return the next time to set */
		ptp_clock_event(fep->ptp_clock, &event);
		ns = timespec64_to_ns(&event.alarm_time);

		printk(KERN_ALERT "fec_ptp_timer_settime: time already passed, now setting to %llu\n", ns);
	}

	spin_unlock_irqrestore(&fep->tmreg_lock, irqsaveflags);
	mutex_unlock(&fep->ptp_clk_mutex);
	return 0;
}

static int fec_ptp_timer_enable(struct ptp_clock_info *ptp, bool enable)
{
	struct fec_enet_private *fep =
			    container_of(ptp, struct fec_enet_private, ptp_caps);
	unsigned long flags;


	if (enable)
	{
		if (!fep->timer_enabled)
		{
			//the pin must have been set as timer and assigned a channel
			if(fep->sdp_config[FEC_SDP_TIMER].func != PTP_PF_TIMER)
				return -EBUSY;

			spin_lock_irqsave(&fep->tmreg_lock, flags);
			fep->timer_enabled = true;
			spin_unlock_irqrestore(&fep->tmreg_lock, flags);

			printk(KERN_ALERT "fec: timer enabled\n");
			return 0;
		}
		/* timer is already enabled */
		return -EINVAL;
	} else {
		if (!fep->timer_enabled)
			return -EINVAL;

		spin_lock_irqsave(&fep->tmreg_lock, flags);
		/* disable timer */
		fec_tc_enable(fep, false, 0);
		fep->timer_enabled = false;
		spin_unlock_irqrestore(&fep->tmreg_lock, flags);

		printk(KERN_ALERT "fec: timer disabled\n");
	}

	return 0;
}

static int fec_ptp_verify_pin(struct ptp_clock_info *ptp, unsigned int pin,
			      enum ptp_pin_function func, unsigned int chan)
{
	//the timer 'pin' can only be a timer, or NONE
	if(pin == FEC_SDP_TIMER)
	{
		if((func == PTP_PF_NONE) || (func == PTP_PF_TIMER))
			return 0;
		return -1;
	}

	//all other pins can be EXTTS, PEROUT, or NONE
	switch (func)
	{
	case PTP_PF_NONE:
	case PTP_PF_EXTTS:
	case PTP_PF_PEROUT:
		break;
	case PTP_PF_PHYSYNC:
	case PTP_PF_TIMER:
		return -1;
	}
	return 0;
}

/**
 * fec_ptp_init
 * @ndev: The FEC network adapter
 *
 * This function performs the required steps for enabling ptp
 * support. If ptp support has already been loaded it simply calls the
 * cyclecounter init routine and exits.
 */

void fec_ptp_init(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct fec_enet_private *fep = netdev_priv(ndev);
	int i;

	static const char *ptpEventNames[]=
	{
			"POSIX timer (alarm) event",
			"ENET_1588_EVENT0_IN", "ENET_1588_EVENT0_OUT",
			"ENET_1588_EVENT1_IN", "ENET_1588_EVENT1_OUT",
			"ENET_1588_EVENT2_IN", "ENET_1588_EVENT2_OUT",
			"ENET_1588_EVENT3_IN", "ENET_1588_EVENT3_OUT"
	};

	fep->ptp_caps.owner = THIS_MODULE;
	snprintf(fep->ptp_caps.name, 16, "fec ptp");

	for (i = 0; i < FEC_NB_SDP; i++)
	{
		struct ptp_pin_desc *ppd = &fep->sdp_config[i];

		strncpy(ppd->name, ptpEventNames[i], sizeof(ppd->name));
		ppd->index = i;
		ppd->func = PTP_PF_NONE;
	}

	fep->ptp_caps.max_adj = 250000000;
	fep->ptp_caps.n_alarm = 1;
	fep->ptp_caps.n_ext_ts = 4;
	fep->ptp_caps.n_per_out = 4;
	fep->ptp_caps.n_pins = FEC_NB_SDP;
	fep->ptp_caps.pps = 0;
	fep->ptp_caps.pin_config = fep->sdp_config;
	fep->ptp_caps.adjfreq = fec_ptp_adjfreq;
	fep->ptp_caps.adjtime = fec_ptp_adjtime;
	fep->ptp_caps.gettime64 = fec_ptp_gettime;
	fep->ptp_caps.settime64 = fec_ptp_settime;
	fep->ptp_caps.enable = fec_ptp_enable;
	fep->ptp_caps.verify = fec_ptp_verify_pin;
	fep->ptp_caps.timersettime = fec_ptp_timer_settime;

	fep->cycle_speed = clk_get_rate(fep->clk_ptp);
	fep->ptp_inc = NSEC_PER_SEC / fep->cycle_speed;
	fep->timer_enabled = false;

	spin_lock_init(&fep->tmreg_lock);

	fec_ptp_start_cyclecounter(ndev);

	INIT_DELAYED_WORK(&fep->time_keep, fec_time_keep);

	fep->ptp_clock = ptp_clock_register(&fep->ptp_caps, &pdev->dev);
	if (IS_ERR(fep->ptp_clock)) {
		fep->ptp_clock = NULL;
		pr_err("ptp_clock_register failed\n");
	}

	schedule_delayed_work(&fep->time_keep, HZ);
}

uint fec_ptp_check_alarm_event(struct fec_enet_private *fep)
{
	u32 val;
	struct ptp_clock_event event;
	u64 ns;

	val = readl(fep->hwp + FEC_TCSR(fep->sdp_config[FEC_SDP_TIMER].chan));

	if (val & FEC_T_TF_MASK)
	{
		fec_tc_enable(fep, false, 0);

		if (fep->timer_enabled)
		{
			event.alarm_time = ns_to_timespec64(timecounter_read(&fep->tc));
			event.type = PTP_CLOCK_ALARM;
			event.index = 0;

			//printk(KERN_ALERT "fec_ptp_check_alarm_event: got timer int sending event ts %d:%d\n", (int)event.alarm_time.tv_sec, event.alarm_time.tv_nsec);

			/* ptp_clock_event will return the next time to set */
			ptp_clock_event(fep->ptp_clock, &event);

			ns = timespec64_to_ns(&event.alarm_time);

			/* set trigger time and enable timer */
			while ((ns !=0) && !fec_tc_enable(fep, true, ns))
			{
				event.type = PTP_CLOCK_ALARM;
				event.index = 0;
				/* ptp_clock_event will return the next time to set */
				ptp_clock_event(fep->ptp_clock, &event);
				ns = timespec64_to_ns(&event.alarm_time);

				printk(KERN_ALERT "fec_ptp_check_alarm_event: time already passed, now setting to %llu\n", ns);
			}
		}
		return 1;
	}

	return 0;
}

/**
 * fec_ptp_check_pps_event
 * @fep: the fec_enet_private structure handle
 *
 * This function check the pps event and reload the timer compare counter.
 */
uint fec_ptp_check_pps_event(struct fec_enet_private *fep)
{
	u32 val;
	u8 channel = fep->pps_channel;
	struct ptp_clock_event event;

	val = readl(fep->hwp + FEC_TCSR(channel));
	if (val & FEC_T_TF_MASK) {
		/* Write the next next compare(not the next according the spec)
		 * value to the register
		 */
		writel(fep->next_counter, fep->hwp + FEC_TCCR(channel));
		do {
			writel(val, fep->hwp + FEC_TCSR(channel));
		} while (readl(fep->hwp + FEC_TCSR(channel)) & FEC_T_TF_MASK);

		/* Update the counter; */
		fep->next_counter = (fep->next_counter + fep->reload_period) & fep->cc.mask;

		event.type = PTP_CLOCK_PPS;
		ptp_clock_event(fep->ptp_clock, &event);
		return 1;
	}

	return 0;
}
