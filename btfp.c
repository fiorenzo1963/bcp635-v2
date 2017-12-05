/*
 * Copyright (c) 2005 Rob Neal hundoj@comcast.net
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/*
 * Driver for Symmetricom bc637PCI-U GPS Synchronized, PCI Time & Frequency
 * Processor.
 *
 * The bc637PCI-U is a PCI bus card that provides a variety of timing services
 * via direct bus access. The bc635PCI is the model without a GPS receiver. The
 * GPS included on the 637 is the Trimble ACE iii.
 *
 * See www.symmetricom.com and www.trimble.com for details.
 *
 * This driver was developed and tested on FreeBSD 5.3, with a bc637PCIU,
 * firmware DT1000 rev 42712, board type 12083.
 *
 * Current interrupt handler code is lacking in several respects. See XXX
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/module.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <machine/bus.h>
#include <sys/bus.h>
#include <machine/resource.h>
#include <sys/rman.h>
#include <sys/pciio.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <sys/lock.h>
#include <sys/condvar.h>	/* cv_init, cv_destroy, etc.. */
#include <sys/mutex.h>
#include <sys/errno.h>
#include <sys/uio.h>

#include "btfp.h"
#include "ACEiii.h"		/* maps for ACE III GPS on bc637 */

/* Prototypes - PCI bus interface routines */

static int btfp_probe(device_t dev);
static int btfp_attach(device_t dev);
static int btfp_detach(device_t dev);
static void btfp_shutdown(device_t dev);
static void btfp_ih(void *dmy);	/* interrupt handler */
/*
 * Prototypes - internal functions
 */
static int btfp_do_command(struct btfp_sc *card);
static void dpr_read(uint8_t * dest, uint16_t offset, uint8_t count);
static void dpr_write(uint8_t * src, uint16_t offset, uint8_t count);
/*
 * Global variables
 */
static devclass_t btfp_devclass;
static uint8_t cd_bufr[MAXBUFR];/* to/fro cd_read/cd_write functions */
static struct btfp_sc *sc;	/* card descriptor, used everywhere */
static volatile uint32_t blabber;	/* control verbosity */
static volatile uint32_t snoozer;	/* spin & wait time constant */

static struct cdevsw btfp_cdevsw = {
	.d_name = "btfp0",
	.d_version = D_VERSION,
	.d_open = btfp_cd_open,
	.d_close = btfp_cd_close,
	.d_read = btfp_cd_read,
	.d_write = btfp_cd_write,
	.d_ioctl = btfp_cd_ioctl,
};
/*
 * Identify driver call point support for the kernel.
 */
static device_method_t btfp_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, btfp_probe),
	DEVMETHOD(device_attach, btfp_attach),
	DEVMETHOD(device_detach, btfp_detach),
	DEVMETHOD(device_shutdown, btfp_shutdown),

	/* Bus interface */
	DEVMETHOD(bus_print_child, bus_generic_print_child),
	DEVMETHOD(bus_driver_added, bus_generic_driver_added),
	{0, 0}
};
/* declare the driver */
static driver_t btfp_driver = {
	"btfp",			/* used for prefix on device name */
	btfp_methods,
	sizeof(struct btfp_sc),
};
/*
 * Device Probe - find the PCI card we know how to handle.
 */
static int
btfp_probe(device_t dev)
{
	if (pci_read_config(dev, PCIR_VENDOR, 2) == BC637_VENDOR_ID &&
	    pci_read_config(dev, PCIR_DEVICE, 2) == BC637_DEVICE_ID)
		return (0);
	else
		return (ENXIO);
}
/*-
 * Attach card:
 * Allocate bus resources, enable memory access to DPRAM.
 * Save bus space tag & handles for r/w access.
 * Get and save DPRAM offset addresses.
 * Setup interrupt handler routine & clear interrupt flags.
 * Set TFP register format to UNIX binary instead of BCD. Just in case.
 * Determine if device includes GPS.
 * If GPS present, enable GPS interrupts, if locked sync TFP RTC. Set options.
 */
static int
btfp_attach(device_t dev)
{
	static uint32_t u32;
	static uint8_t outstring[MAXBUFR];

	sc = device_get_softc(dev);
	if (sc == NULL) {
		device_printf(dev, "softc allocation failed \n");
		return (ENXIO);
	}
	sc->dev = dev;

	snoozer = SNOOZER;
	blabber = NO_BLAB;

	u32 = PCIR_BAR(0);
	sc->bar0res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &u32, RF_ACTIVE);
	if (sc->bar0res == NULL) {
		device_printf(dev, "cannot map bar0 register\n");
		return (ENXIO);
	}
	sc->bst0 = rman_get_bustag(sc->bar0res);
	sc->bsh0 = rman_get_bushandle(sc->bar0res);

	u32 = PCIR_BAR(1);	/* avoid cast & compiler nastygram */
	sc->bar1res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &u32, RF_ACTIVE);

	if (sc->bar1res == NULL) {
		device_printf(dev, "cannot map bar1 register\n");
		return (ENXIO);
	}
	sc->bst1 = rman_get_bustag(sc->bar1res);
	sc->bsh1 = rman_get_bushandle(sc->bar1res);

	u32 = pci_enable_io(dev, SYS_RES_MEMORY);
	if (u32 != 0) {
		device_printf(dev, "memory mapping failed\n");
		bus_release_resource(dev, SYS_RES_MEMORY, PCIR_BAR(0), sc->bar0res);
		bus_release_resource(dev, SYS_RES_MEMORY, PCIR_BAR(1), sc->bar1res);
		return (u32);
	}
	/*-
	* Interrupt setup processing.
	* Go get an IRQ assignment from PCI parent & a mutex for serialization.
	* Disable all TFP to PCI interrupt promotion, reset flags, and assign
	* interrupt handler routine.
	*/
	u32 = 0;
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &u32,
	    RF_ACTIVE | RF_SHAREABLE);

	if (sc->irq == NULL) {
		device_printf(dev, "cannot map IRQ line\n");
		return (ENXIO);
	}
	mtx_init(&sc->mutex, device_get_nameunit(dev), "btfp driver", MTX_DEF);
	cv_init(&sc->condvar, "btfp bc637PCIu");

	u32 = DR_READ(TFP_REG_MASK) & ~TFP_IR_ALL;
	DR_WRITE(TFP_REG_MASK, u32);

	u32 = DR_READ(TFP_REG_INTSTAT) & ~TFP_IR_ALL;
	DR_WRITE(TFP_REG_INTSTAT, u32);

	u32 = bus_setup_intr(dev, sc->irq, INTR_TYPE_MISC, btfp_ih, sc,
	    &sc->cookiep);

	if (u32 != 0) {
		device_printf(dev, "cannot establish IRQ handler\n");
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq);
		mtx_destroy(&sc->mutex);
		cv_destroy(&sc->condvar);
		return (u32);
	}
/*-
 * NB: DPRAM phsical map is :
 * 	+0000 Year
 *	+0002 GPS packet area
 *	+0082 Output Area
 *	+0102 Input Area
 * for this particular firmware release level. The actual offsets of the
 * starting addresses are retrieved from end of DPRAM, towards zero.
 */
	sc->Inarea = DP_READ(DPRAM_LEN - 2);
	sc->Inarea = sc->Inarea << 8;
	sc->Inarea += DP_READ(DPRAM_LEN - 1);

	sc->Outarea = DP_READ(DPRAM_LEN - 4);
	sc->Outarea = sc->Outarea << 8;
	sc->Outarea += DP_READ(DPRAM_LEN - 3);

	sc->GPSarea = DP_READ(DPRAM_LEN - 6);
	sc->GPSarea = sc->GPSarea << 8;
	sc->GPSarea += DP_READ(DPRAM_LEN - 5);

	sc->YRarea = DP_READ(DPRAM_LEN - 8);
	sc->YRarea = sc->YRarea << 8;
	sc->YRarea += DP_READ(DPRAM_LEN - 7);
/*
 * Set time register format to Unix time, instead of BCD.
 * This should be the powerup state, but be certain
 */
	DP_WRITE(sc->Inarea, TFP_TIMEREG_FMT);
	DP_WRITE(sc->Inarea + 1, UNIX_TIME);
	btfp_do_command(sc);

/* get model id, and set GPS status */

	DP_WRITE(sc->Inarea, TFP_REQUEST_DATA);
	DP_WRITE(sc->Inarea + 1, TFP_MODEL);
	btfp_do_command(sc);

	sc->hasGPS = (DP_READ(sc->Outarea + 5) == '7');

	if (sc->hasGPS) {
		sc->pktrdy = 0;

		/* enable GPS packet ready PCI interrupts */
		DR_WRITE(TFP_REG_MASK, TFP_IR_GPSPKT);

		/* get status and determine if flywheeling or locked */
		u32 = DR_READ(TFP_REG_TIMEREQ);	/* latches TIME0 */
		u32 = DR_READ(TFP_REG_TIME0);

		if (u32 & TFP_MR_FLYWHEEL)
			device_printf(dev,
			    "GPS flywheeling, no time lock\n");
		else {
			/* update TFP RTC to GPS */
			DP_WRITE(sc->Inarea, TFP_SYNC_RTC);
			btfp_do_command(sc);
			device_printf(dev, "GPS is locked,TFP onboard RTC synchronized\n");
		}

		/* verify GPS is providing UTC time not GPS time */

		DP_WRITE(sc->Inarea, TFP_REQUEST_DATA);
		DP_WRITE(sc->Inarea + 1, TFP_UTC_INFO_CTRL);
		btfp_do_command(sc);

		bzero(outstring, MAXBUFR);
		dpr_read(outstring, sc->Outarea, 8);

		if (outstring[0] == TFP_UTC_INFO_CTRL) {	/* verify cmd response */
			if (outstring[1] & GPS_TIME) {
				/* not UTC, change it */
				device_printf(sc->dev, "Setting GPS unit to return UTC\n");
				DR_WRITE(TFP_REG_ACK, TFP_GPSPKT_RDY);
				DP_WRITE(sc->Inarea, TFP_GPS_TIMEFMT);
				DP_WRITE(sc->Inarea + 1, UTC_TIME);
				btfp_do_command(sc);
			} else
				device_printf(sc->dev, "GPS providing UTC time\n");
		} else
			device_printf(sc->dev,
			    "GPS UTC query command error, GPS unit is suspect.\n");
	}
	return (0);
}
/*
 * Detach driver from bus, return bus resources.
 */
static int
btfp_detach(device_t dev)
{
	struct btfp_sc *sc = device_get_softc(dev);

	bus_teardown_intr(dev, sc->irq, sc->cookiep);

	bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq);
	mtx_destroy(&sc->mutex);
	cv_destroy(&sc->condvar);

	pci_disable_io(dev, SYS_RES_MEMORY);

	bus_release_resource(dev, SYS_RES_MEMORY, PCIR_BAR(0), sc->bar0res);
	bus_release_resource(dev, SYS_RES_MEMORY, PCIR_BAR(1), sc->bar1res);
	if (blabber > NO_BLAB) {
		device_printf(sc->dev, "TFP device detached from bus\n");
	}
	return (0);
}
static void
btfp_shutdown(device_t dev)
{
	struct btfp_sc *sc = device_get_softc(dev);
	if (sc == NULL)
		device_printf(dev, "Why are we here? Bye bye\n");
}
/*
 * Handle PCI interrupts received on the IRQ we were assigned.
 * NB: we get called by everybody that yanks on this shared interrupt, so don't
 * futz around.
 */
static void
btfp_ih(void *dmy)
{
	struct btfp_sc *sc = dmy;
	static uint32_t intr;

	intr = DR_READ(TFP_REG_MASK) & DR_READ(TFP_REG_INTSTAT) & TFP_IR_ALL;
	if (intr == 0)
		return;
/*
 * TFP is knocking, see what it wants.
 */
	if (blabber >= BLAB_INTERRUPT) {
		device_printf(sc->dev, "ih entered intr is %#x\n", intr);
	}
	BTFP_LOCK(sc);

	switch (intr) {

	case TFP_IR_EVENTINP:	/* event input trigger */

		DR_WRITE(TFP_REG_INTSTAT, TFP_IR_EVENTINP);
		/* XXX should be something useful */
		break;

	case TFP_IR_PERIODOUT:	/* heartbeat output */

		DR_WRITE(TFP_REG_INTSTAT, TFP_IR_PERIODOUT);
		/* XXX should be something useful */
		break;

	case TFP_IR_STROBE:	/* strobe input event */

		DR_WRITE(TFP_REG_INTSTAT, TFP_IR_STROBE);
		/* XXX should be something useful */
		break;

	case TFP_IR_1PPSOUT:	/* pulse-per-second from GPS */

		DR_WRITE(TFP_REG_INTSTAT, TFP_IR_1PPSOUT);
		/* XXX should be something useful */
		break;

	case TFP_IR_GPSPKT:	/* GPS packet delivered */

		DR_WRITE(TFP_REG_INTSTAT, TFP_IR_GPSPKT);	/* ack interrupts */
		DR_WRITE(TFP_REG_ACK, TFP_GPSPKT_RDY);

		if (blabber >= BLAB_GPS) {
			device_printf(sc->dev, "GPS packet %#x len %#x is ready\n",
			    DP_READ(sc->GPSarea + 1), DP_READ(sc->GPSarea));
		}
		sc->pktrdy = 1;	/* indicate packet ready */

		cv_broadcast(&sc->condvar);	/* wake up waiters */

		break;

	default:		/* shouldn't be here */
		DR_WRITE(TFP_REG_INTSTAT, 0xffffffff);	/* bfmi */
		device_printf(sc->dev, "btfp_ih called in error %x \n", intr);
		break;
	}
	if (blabber >= BLAB_INTERRUPT) {
		device_printf(sc->dev,
		    "xit ih mask %#x intstat %#x ack %#x pktrdy %#x\n",
		    DR_READ(TFP_REG_MASK), DR_READ(TFP_REG_INTSTAT),
		    DR_READ(TFP_REG_ACK), sc->pktrdy);
	}
	BTFP_UNLOCK(sc);
	return;
}
/*
 * Write count bytes from string src[] to offset in DPRAM in context sc
 */
static void
dpr_write(uint8_t * src, uint16_t offset, uint8_t count)
{
	static uint16_t i;

	for (i = 0; i < count; i++) {
		DP_WRITE(i + offset, src[i]);
	}

	return;
}
/*
 * Read count bytes from offset in DPRAM to string dest[] in context sc
 */
static void
dpr_read(uint8_t * dest, uint16_t offset, uint8_t count)
{
	static uint16_t i;

	for (i = 0; i < count; i++) {
		dest[i] = bus_space_read_1(sc->bst1, sc->bsh1, i + offset);
	}

	return;
}
/*
 * Issue a command to the TFP MPU and wait for it to process.
 * NB: We grab a mutex here, so you can't call this while locked.
 */
static int
btfp_do_command(struct btfp_sc *sc)
{
	static uint32_t u32, gpscmd;
	static int rc;
	static uint8_t cmd[2];

	BTFP_LOCK(sc);
	rc = 0;
	cmd[0] = DP_READ(sc->Inarea);
	cmd[1] = DP_READ(sc->Inarea + 1);

	if (blabber >= BLAB_CALLS) {
		uprintf("command %#x, %#x sent to tfp\n", cmd[0], cmd[1]);
	}

	gpscmd = ((cmd[0] == 0x30) || (cmd[0] == 0x31) || (cmd[0] == 0x32));

	if (gpscmd == 1)
		DR_WRITE(TFP_REG_ACK, TFP_GPSPKT_RDY);

	DR_WRITE(TFP_REG_ACK, TFP_ACK_USRCMD);	/* clear response bit */
	DR_WRITE(TFP_REG_ACK, TFP_MPUCMD_RDY);	/* tell TFP do command */

	for (u32 = 1; u32 < snoozer; u32++) {
		if ((DR_READ(TFP_REG_ACK) & TFP_ACK_USRCMD))
			break;
	}
	if ((gpscmd == 1) && (u32 < snoozer)) {	/* see if gpscmd nonresp */
		if (DP_READ(sc->GPSarea) == 0x00 ) {
			rc = -2;
			uprintf("Gps cmd %x %x did not return data\n", cmd[0], cmd[1]);
		}
	}

	if (u32 >= snoozer) {
		rc = ~0;
		device_printf(sc->dev, "Command %#x %#x timed out!! count %#x\n",
		    cmd[0], cmd[1], u32);
		uprintf("btfp0: Command %#x %#x timed out!! count %#x\n",
		    cmd[0], cmd[1], u32);
	}
	BTFP_UNLOCK(sc);
	return (rc);
}
/*
 * btfp_cd_xxxx routines, char device function handlers
 */
static int
btfp_cd_open(struct cdev *dev, int oflags, int devtype, struct thread *p)
{
	if (blabber >= BLAB_CALLS)
		uprintf("entered btfp_cd_open\n");
	return (0);
}
static int
btfp_cd_close(struct cdev *dev, int fflag, int devtype, struct thread *p)
{
	if (blabber >= BLAB_CALLS)
		uprintf("entered btfp_cd_close\n");
	return (0);
}
/*
 * Read a GPS packet from the DPRAM GPS packet area and send it to userland
 */
static int
btfp_cd_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	static uint8_t gps_pktlen;

	if (blabber > BLAB_CALLS) {
		uprintf("btfp0: entered btfp_cd_read\n");
	}
	if (sc->hasGPS == 0) {
		uio->uio_resid = 0;	/* don't come back, now, y'hear? */
		uprintf("btfp0: No GPS on this TFP, read not permitted\n");
		return (ENXIO);
	}
	BTFP_LOCK(sc);

	if (sc->pktrdy == 0) {
		if (EWOULDBLOCK == cv_timedwait(&sc->condvar, &sc->mutex, snoozer / 10)) {
			if (blabber > NO_BLAB) {
				device_printf(sc->dev, "timeout gps packet wait\n");
				uprintf("btfp0: timeout gps packet wait\n");
			}
			BTFP_UNLOCK(sc);
			uio->uio_resid = 0;
			return (ENXIO);
		}
	}
	/* we have a packet ready, and we expected one, give it away */
	sc->pktrdy = 0;		/* indicate packet picked up */

	bzero(cd_bufr, MAXBUFR);

	gps_pktlen = DP_READ(sc->GPSarea);

	if (gps_pktlen == 0) {	/* TFP returned null packet, rq error */
		BTFP_UNLOCK(sc);
		uio->uio_resid = 0;
		return (ENXIO);
	} else {
		gps_pktlen += 1;/* get the cid byte too */
		dpr_read(cd_bufr, sc->GPSarea, gps_pktlen);

		BTFP_UNLOCK(sc);/* can't hold mutex & call uiomove */

		if (EFAULT == (uiomove(cd_bufr, gps_pktlen, uio))) {
			device_printf(sc->dev, "uiomove cd_read EFAULTED \n");
			return (EFAULT);
		}
	}
	return (0);
}
/*
 * Write a GPS command stream to the TFP GPS receiver.
 */
static int
btfp_cd_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	static int rc;
	static uint8_t len;
	static struct ACE_prq_32 *ptr;	/* map packet request */

	if (blabber >= BLAB_CALLS) {
		uprintf("btfp0: entered btfp_cd_write\n");
	}
	if (sc->hasGPS == 0) {
		uprintf("btfp0: No GPS on this TFP, write not permitted\n");
		return (ENXIO);
	}
	bzero(cd_bufr, MAXBUFR);

	rc = uiomove(cd_bufr, uio->uio_resid, uio);

	if (rc == EFAULT) {
		if (blabber > NO_BLAB) {
			device_printf(sc->dev, "uiomove EFAULTED in write \n");
			uprintf("btfp0: uiomove EFAULTED in write \n");
		}
		uio->uio_resid = 0;
		return (ENXIO);
	}
	/* check command packet to see if it is really GPS command */

	ptr = (struct ACE_prq_32 *)cd_bufr;	/* map the command stream */

	switch (ptr->cid) {

	case TFP_GPS_SENDPKT:
		len = ptr->report_len + 1;	/* +1 for cid */
		if (blabber >= BLAB_GPS) {
			uprintf("btfp0: GPS_REQ %#x, pktid %#x, pktlen %#x\n",
			    ptr->cid, ptr->report_id, ptr->report_len);
		}
		break;

	case TFP_GPS_REQPKT:
		len = 2;	/* cid, pid only */
		if (blabber >= BLAB_GPS) {
			uprintf("btfp0: GPS_REQ 0x31 %#x\n", cd_bufr[1]);
		}
		break;

	case TFP_GPS_MANUAL:
		if (blabber >= BLAB_GPS) {
			device_printf(sc->dev,
			    "GPS_REQ cid %x rppl %x rspid %x rqplen %x rqpid %x \n",
			    ptr->cid, ptr->report_len, ptr->report_id,
			    ptr->cmdpkt_len, ptr->cmdpkt_id);

			uprintf("btfp0: GPS_REQ cid %x rppl %x rspid %x rqplen %x rqpid %x \n",
			    ptr->cid, ptr->report_len, ptr->report_id,
			    ptr->cmdpkt_len, ptr->cmdpkt_id);
		}
		len = ptr->cmdpkt_len + 3;	/* cid, rsp pktid, len */
		break;

	default:		/* don't do these commands */
		if (blabber > NO_BLAB) {
			uprintf("btfp0: bad GPScmd %#x \n", cd_bufr[0]);
		}
		return (ENXIO);
	}
	/* move user supplied command stream into DPRAM */

	rc = len;

	BTFP_LOCK(sc);		/* diddle flags, serialize */

	dpr_write(cd_bufr, sc->Inarea, len);

	DR_WRITE(TFP_REG_ACK, TFP_GPSPKT_RDY);	/* clear by setting! */
	DR_WRITE(TFP_REG_INTSTAT, TFP_IR_GPSPKT);	/* so TFP will ack done */

	BTFP_UNLOCK(sc);	/* unlock before calling do_command() */

	btfp_do_command(sc);

	uio->uio_resid = 0;	/* write is over, no matter what */
	return (0);
}
/*
 * The majority of all userland calls are handled here. GPS r/w support is
 * provided via the read/write file calls.
 */
static int
btfp_cd_ioctl(struct cdev *dev, u_long cmd, caddr_t arg, int flag, struct thread *td)
{
	static struct btfp_inarea *iptr;
	static struct timereg *timereg_p;
	static uint32_t callno, rc, tmp, u32, *u32_p;
	static uint8_t *u8_p, len;
	static union btfp_ioctl_out *btfpctl_p;

	/* cast some pointers to arg, for compiler happiness */
	iptr = (struct btfp_inarea *)arg;
	timereg_p = (struct timereg *)arg;
	u8_p = (uint8_t *) arg;

	callno = cmd & 0xff;	/* mask off the ioctl control info */

	if (blabber >= BLAB_CALLS) {
		uprintf("btfp0: ioctl cmd %x, arg %x, callno %x\n",
		    (uint32_t) cmd, (uint32_t) * arg, callno);
	}

	len = 0;
	rc = 0;

	switch (callno) {
	case TFP_LATCH_TIMEREG:
		/* Latch current time into TIME0 & TIME1 registers. */
		DR_READ(TFP_REG_TIMEREQ);
		break;

	case TFP_LATCH_EVENTREG:
		/* Latch current time into EVENT0 & EVENT1 registers. */
		DR_READ(TFP_REG_EVENTREQ);
		break;

	case TFP_UNLOCK_CAPTURE:
		/* Reset the EVENT0/1 register lockout. The next trigger event
		 * time will be captured. Does not effect interrupt
		 * generation. Control capture lockout with CAPLOCK_ENABLE /
		 * CAPLOCK_DISABLE. */
		DR_READ(TFP_REG_UNLOCK);
		break;

	case TFP_POR:		/* software reset of TFP */
		dpr_write(u8_p, sc->Inarea, 1);
		rc = btfp_do_command(sc);
		uprintf("TFP software reset initiated, rc = %#x\n", rc);
		break;
	case TFP_FORCE_JAMSYNC:
		/* FALLTHROUGH */
	case TFP_SYNC_RTC:
		/* FALLTHROUGH */
	case TFP_BATT_DISCON:
		dpr_write(u8_p, sc->Inarea, 1);
		rc = btfp_do_command(sc);
		break;

	case TFP_REQUEST_DATA:
		/* Request data from TFP that is not available via device
		 * registers. First byte of the data returned is the response
		 * type byte. */

		switch (iptr->subcmd) {

		case TFP_TIMEMODE:
			/* FALLTHROUGH */
		case TFP_TIMEREG_FMT:
			/* FALLTHROUGH */
		case TFP_INPUT_CODE_FMT:
			/* FALLTHROUGH */
		case TFP_CODE_MODULATION:
			/* FALLTHROUGH */
		case TFP_OUTPUT_CODE_FMT:
			/* FALLTHROUGH */
		case TFP_CLOCK_SOURCE:
			/* FALLTHROUGH */
		case TFP_JAMSYNC_CTRL:
			/* FALLTHROUGH */
		case TFP_BATTERY_STATUS:
			/* FALLTHROUGH */
		case TFP_LOCALTZ_OBS_QRY:
			len = 2;
			break;

		case TFP_YEAR:
			/* FALLTHROUGH */
		case TFP_LOAD_DAC:
			/* FALLTHROUGH */
		case TFP_ASSEMBLY:
			/* FALLTHROUGH */
		case TFP_HDW_FAB:
			len = 3;
			break;

		case TFP_PPO:
			/* FALLTHROUGH */
		case TFP_LEAP_SETTING:
			/* FALLTHROUGH */
		case TFP_CLOCK_SLEW:
			len = 6;
			break;

		case TFP_PROP_DELAY:
			/* FALLTHROUGH */
		case TFP_SERIAL:
			len = 5;
			break;

		case TFP_UTC_INFO_CTRL:
			len = 8;
			break;

		case TFP_GEN_OFFSET:
			/* FALLTHROUGH */
		case TFP_LOCAL_TZ:
			len = 4;
			break;

		case TFP_OSCDISCP_CTRL:
			len = 10;
			break;

		case TFP_REVISION:
			len = 12;
			break;

		case TFP_MODEL:
			len = 9;
			break;

		default:
			rc = ENOTTY;
		}		/* end TFP_REQUEST_DATA switch */

		dpr_write(u8_p, sc->Inarea, 2);	/* write cmd/subcmd */
		rc = btfp_do_command(sc);
		dpr_read(u8_p, sc->Outarea, len);
		break;

/*
 * end TFP_REQUEST_DATA
 */
	case TFP_READ_TIME:
		/* Latch and return TIME0 and TIME1 registers, providing
		 * current time. Note: this call point is used by NTP
		 * reference clock driver 16. */
		tmp = DR_READ(TFP_REG_TIMEREQ);
		timereg_p->time0 = DR_READ(TFP_REG_TIME0);
		timereg_p->time1 = DR_READ(TFP_REG_TIME1);
		break;

	case TFP_READ_EVENTREG:
		DR_READ(TFP_REG_EVENTREQ);
		timereg_p->time0 = DR_READ(TFP_REG_EVENT0);
		timereg_p->time1 = DR_READ(TFP_REG_EVENT1);
		break;

	case TFP_SET_REG_CONTROL:
		u32 = (uint32_t) * arg;
		u32 &= ~TFP_CR_RESERVED;
		DR_WRITE(TFP_REG_CONTROL, u32);
		break;

	case TFP_FETCH_REG_CONTROL:
		u32_p = (uint32_t *) arg;
		*u32_p = DR_READ(TFP_REG_CONTROL);
		break;

	case TFP_SET_REG_MASK:
		u32 = (uint32_t) * arg;
		u32 &= TFP_IR_ALL;
		DR_WRITE(TFP_REG_MASK, u32);
		break;

	case TFP_FETCH_REG_MASK:
		u32_p = (uint32_t *) arg;
		*u32_p = DR_READ(TFP_REG_MASK);
		break;

	case TFP_SET_REG_INTSTAT:
		u32 = (uint32_t) * arg;
		u32 &= TFP_IR_ALL;
		DR_WRITE(TFP_REG_INTSTAT, u32);
		break;

	case TFP_FETCH_REG_INTSTAT:
		u32_p = (uint32_t *) arg;
		*u32_p = DR_READ(TFP_REG_INTSTAT);
		break;

	case TFP_BTFPCTL:
		btfpctl_p = (union btfp_ioctl_out *)arg;
		switch (btfpctl_p->btfpctl.id) {
		case BLABBER_LESS:
			if (blabber > 0) {
				blabber -= 1;
				uprintf("btfp0: blabber decreased to %#x\n", blabber);
			}
			break;
		case BLABBER_MORE:
			if (blabber < BLABBERMAX) {
				blabber += 1;
				uprintf("btfp0: blabber increased to %#x\n", blabber);
			}
			break;
		case BLABBER_NONE:
			blabber = NO_BLAB;
			break;
		case BLABBER_LEVEL:
			btfpctl_p->btfpctl.u32 = blabber;
			break;
		case SNOOZER_FETCH:
			btfpctl_p->btfpctl.u32 = snoozer;
			break;
		case SNOOZER_SET:
			if (btfpctl_p->btfpctl.u32 > 0)
				snoozer = btfpctl_p->btfpctl.u32;	/* minor noiding */
			break;
		default:
			rc = ENOTSUP;
			break;
		}
		break;

	case TFP_TIMEMODE:
		/* Select timing mode input reference: GPS, PPS, etc. */
		switch (iptr->subcmd) {
		case TIMEMODE_GPS:
			if (sc->hasGPS == 0) {
				rc = ENOTSUP;
				break;
			}
			/* FALLTHROUGH */
		case TIMEMODE_CODE:
			/* FALLTHROUGH */
		case TIMEMODE_FREERUN:
			/* FALLTHROUGH */
		case TIMEMODE_PPS:
			/* FALLTHROUGH */
		case TIMEMODE_RTC:
			len = 2;
			break;
		default:
			rc = ENOTSUP;
			break;
		}
		break;		/* end TFP_TIMEMODE */

	case TFP_TIMEREG_FMT:
		/* Set time register format: BCD or binary Unix */
		switch (iptr->subcmd) {
		case UNIX_TIME:
			/* FALLTHROUGH */
		case BCD_TIME:
			len = 2;
			break;
		default:
			rc = ENOTTY;
			break;
		}
		break;		/* end TFP_SET_TIMEREG_FMT */

	case TFP_MAJOR_TIME:
		/* Set Major time. Not needed in modes 0,3,6. Readable in 1sec
		 * epoch following the time load. Issue before .8 into current
		 * 1 sec epoch. Format depends on 0x11 cmd time format
		 * selection. */
		len = 9;
		break;

	case TFP_YEAR:
		len = 3;
		break;

	case TFP_PPO:
		len = 6;
		break;

	case TFP_INPUT_CODE_FMT:
		switch (iptr->subcmd) {
		case IRIG_A:
			/* FALLTHROUGH */
		case IRIG_B:
			/* FALLTHROUGH */
		case IEEE_1344:
			/* FALLTHROUGH */
		case NASA36:
			len = 2;
			break;
		default:
			rc = ENOTTY;
			break;
		}
		break;		/* end TFP_SET_INPUT_CODE */

	case TFP_CODE_MODULATION:
		switch (iptr->subcmd) {
		case AM_SINE:
			/* FALLTHROUGH */
		case DCLS_PCM:
			len = 2;
			break;
		default:
			rc = ENOTTY;
			break;
		}
		break;		/* end TFP_CODE_MODULATION */

	case TFP_PROP_DELAY:
		len = 5;
		break;

	case TFP_UTC_INFO_CTRL:
		switch (iptr->subcmd) {
		case UTC_TIME:
			/* FALLTHROUGH */
		case GPS_TIME:
			len = 2;
			break;
		default:
			rc = ENOTTY;
			break;
		}
		break;

	case TFP_OUTPUT_CODE_FMT:
		switch (iptr->subcmd) {
		case IRIG_B:
			/* FALLTHROUGH */
		case IEEE_1344:
			len = 2;
			break;
		default:
			rc = ENOTTY;
			break;
		}
		break;

	case TFP_GEN_OFFSET:
		len = 4;
		break;

	case TFP_LOCAL_TZ:
		len = 4;
		break;

	case TFP_LEAP_SETTING:
		len = 6;
		break;

	case TFP_CLOCK_SOURCE:
		switch (iptr->subcmd) {
		case INTERNAL:
			/* FALLTHROUGH */
		case EXTERNAL:
			len = 2;
		default:
			rc = ENOTSUP;
			break;
		}
		break;

	case TFP_JAMSYNC_CTRL:
		len = 2;
		break;

	case TFP_LOAD_DAC:
		len = 3;
		break;

	case TFP_DISCIPLIN_GAIN:
		len = 3;
		break;

	case TFP_CLOCK_SLEW:
		len = 6;
		break;

	case TFP_GPS_SENDPKT:
		/* FALLTHROUGH */
	case TFP_GPS_REQPKT:
		/* FALLTHROUGH */
	case TFP_GPS_MANUAL:
		/* GPS interface supported via read/write of device, not ioctl */
		rc = ENOTSUP;
		break;

	case TFP_GPS_TIMEFMT:
		if (sc->hasGPS == 0) {
			rc = ENOTSUP;
			break;
		}
		switch (iptr->subcmd) {
		case UTC_TIME:
			/* FALLTHROUGH */
		case GPS_TIME:
			len = 2;
			break;
		default:
			rc = ENOTSUP;
			break;
		}
		break;

	case TFP_GPS_STATION_MODE:
		/* set GPS station mode. Default is to switch to station mode
		 * after lock is acquired */
		if (sc->hasGPS == 0) {
			rc = ENOTSUP;
			break;
		}
		switch (iptr->subcmd) {
		case STATION:
			/* FALLTHROUGH */
		case NO_STATION:
			len = 2;
			break;
		default:
			rc = ENOTSUP;
			break;
		}
		break;

	case TFP_OBS_LOCAL_TZ:
		/* Observe local time or no. Default is enabled. */
		switch (iptr->subcmd) {
		case LOCAL:
			/* FALLTHROUGH */
		case NO_LOCAL:
			len = 2;
			break;
		default:
			rc = ENOTTY;
			break;
		}
		break;

	case TFP_AUTO_YEAR_INCR:
		switch (iptr->subcmd) {
		case AUTO_INCR:
			/* FALLTHROUGH */
		case NO_AUTO_INCR:
			len = 2;
			break;
		default:
			rc = ENOTTY;
			break;
		}
		break;

	default:
		return (ENOTTY);
	}

	if (len != 0) {
		dpr_write(u8_p, sc->Inarea, len);
		rc = btfp_do_command(sc);
	}
	return (rc);
}				/* end btfp_cd_ioctl() function */
/*
 * Module load/unload event handler.
 */
static int
btfp_load(struct module *m, int event, void *arg)
{
	static char *devname = "btfp0";	/* XXX hardcoded, blech */
	static struct cdev *b_tfp_dev;	/* devfs make/destroy  */

	switch (event) {

	case MOD_LOAD:
		b_tfp_dev = make_dev(&btfp_cdevsw, 0, UID_ROOT, GID_WHEEL,
		    DEV_PERMISSIONS, devname);
		uprintf("bc637PCI-U Time & Frequency Processor driver loaded.\n");
		uprintf("bc637PCI-U TFP device is %s\n", devname);
		break;

	case MOD_UNLOAD:
		destroy_dev(b_tfp_dev);
		uprintf("bc637PCI-U Time & Frequency Processor driver unloaded.\n");
		break;

	case MOD_SHUTDOWN:
		destroy_dev(b_tfp_dev);
		break;

	default:
		return (EINVAL);
	}
	return (0);
}
/*
 * Declare driver module to the kernel
 */
DRIVER_MODULE(btfp, pci, btfp_driver, btfp_devclass, btfp_load, 0);
