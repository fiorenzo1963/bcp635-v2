/*
 * Copyright (c) 2005 Rob Neal	hundoj@comcast.net
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
 * btfp.h
 * Header file for the Symmetricom bc637-U PCI card, with GPS option
 * 
 * NB: the bc635/637 is a big-endian device. Intel is little-endian. Enjoy.
 */

/* maximum buffer length for read/write */
#define MAXBUFR 		512	/* 255 * 2 (pkt in/out) + 1 * 2 (pkt id in/out) */

#ifdef _KERNEL
/* 
 * structure describing the TFP card to the driver
 */
struct btfp_sc {
	device_t			dev;		/* card pointer							*/
	struct resource 	*bar0res;	/* Control regs BAR0 based				*/
	bus_space_handle_t	bsh0;		/* BAR0 bus space handle				*/
	bus_space_tag_t		bst0;		/* BAR0 bus space tag					*/
	struct resource		*bar1res;   /* DPRAM BAR1 based						*/
	bus_space_handle_t	bsh1;		/* BAR1 bus space handle				*/
	bus_space_tag_t		bst1;		/* BAR1 bus tag handle					*/
	struct resource  	*irq;		/* IRQ line								*/
	struct mtx			mutex;		/* locking mutex						*/
	struct mtx			spin_mutex;	/* spin mutex						*/
	struct cv			condvar;	/* locking condition variable			*/
	void				*cookiep;	/* IRQ cookie - teardown				*/
	uint16_t 			Inarea;		/* Input Area offset					*/
	uint16_t			Outarea;	/* Output Area offset					*/
	uint16_t 			GPSarea;	/* GPS Packet Area offset				*/
	uint16_t 			YRarea;	 	/* Year Area offset						*/
	uint8_t				hasGPS; 	/* GPS present flag						*/
	uint8_t				pktrdy;		/* GPS packet ready for pickup			*/
	uint8_t				timeformat;
	uint8_t			cd_bufr[MAXBUFR];/* to/fro cd_read/cd_write functions */
};
/* 
 * char device driver stuff 
 */
static d_open_t		btfp_cd_open;
static d_close_t	btfp_cd_close;
static d_read_t		btfp_cd_read;
static d_write_t	btfp_cd_write;
static d_ioctl_t	btfp_cd_ioctl;

/* read a device register off BAR0 */
#define DR_READ(sc, reg)	(bus_space_read_4((sc)->bst0, (sc)->bsh0, (reg)))

/* write a device register off BAR0 */
#define DR_WRITE(sc, reg, data)	(bus_space_write_4((sc)->bst0, (sc)->bsh0, (reg), (data)))

/* write a byte to a DPRAM offset */
#define DP_WRITE(sc, off, byt)  (bus_space_write_1((sc)->bst1, (sc)->bsh1, (off), (byt)))

/* read a byte from a DPRAM offset */
#define DP_READ(sc, off)		(bus_space_read_1((sc)->bst1, (sc)->bsh1, (off)))

#define BTFP_LOCK(_sc)          mtx_lock(&(_sc)->mutex)
#define BTFP_UNLOCK(_sc)        mtx_unlock(&(_sc)->mutex) 

#define CDEV_GET_SOFTC(x)	((x)->si_drv1)

#define BC637_VENDOR_ID			0x12e2 
#define BC637_DEVICE_ID			0x4013 
#define DPRAM_LEN				0x1000 		
#define EOF						-1
#define DEV_PERMISSIONS			0660	
#define NO_BLAB 				0
#define	BLAB_GPS				1
#define BLAB_CALLS				2
#define BLAB_INTERRUPT			3
#define BLABBERMAX				4
#endif /* _KERNEL */ 
/******************************************************************************
 *	Endian-ness requires attention. Fair warning.
 *  "man 9 byteorder" is your friend.
 *****************************************************************************/
/* 
 *	Various structures used to map card objects
 */
struct  btfp_inarea {
		uint8_t				cmd;		/* command byte to TFP  */
		uint8_t 			subcmd;		/* subfunction code */	
}__attribute__((packed));
struct	model	{
		uint8_t	id;
		uint8_t	model[8];
}__attribute__((packed));
/*
 * time0/1 register pair: TIME, EVENT, STROBE
 */
struct	timereg		{
		uint32_t			time0;
		uint32_t			time1;
}__attribute__((packed));
struct	timemode	{					/* input time mode */
		uint8_t		id;			
		uint8_t		mode;
}__attribute__((packed));
struct	timefmt		{					/* time register format */
		uint8_t		id;
		uint8_t		format;
}__attribute__((packed));
struct	ppo			{					/* programmable periodic output */
		uint8_t		id;
		uint8_t		sync;
		uint16_t	divider_1;
		uint16_t	divider_2;
}__attribute__((packed));
struct	timecode	{					/* timecode format */
		uint8_t		id;
		uint8_t		timecode;
}__attribute__((packed));
struct	timecodemodulation	{				/* timecode modulation type (M or D) */
		uint8_t		id;
		uint8_t		modulation;
}__attribute__((packed));
struct  propdelay	{					/* propagation delay */
		uint8_t		id;
		int32_t		propdelay;
}__attribute__((packed));
struct	jamsync		{
		uint8_t		id;
		uint8_t		ctrl;
}__attribute__((packed));
struct	firmware	{
		uint8_t		id;
		uint8_t		revision[11];
}__attribute__((packed));
struct	assembly		{
		uint8_t		id;
		uint16_t	assembly;
}__attribute__((packed));
struct	serial		{
		uint8_t		id;
		uint8_t		sn[4];						/* endian-ness again :-) */
}__attribute__((packed));
struct	btfpctl		{							/* driver control stuff */
		uint8_t		id;
		uint32_t	u32;
}__attribute__((packed));
struct  set_unixtime	{
		uint8_t		id;
		uint32_t	unix_time;
		uint32_t	zero_filler;
}__attribute__((packed));
union btfp_ioctl_out	{						/* best declared volatile */
		struct		btfp_inarea	inarea;			/* pass cmd/subcmd in */
		struct		timereg		timereg;		/* card registers */
		uint32_t	ctrlreg;				/* control register */
		uint32_t	intmask;				/* PCI interrupt mask */
		uint32_t	intstat;				/* TFP interrupt status */
		struct		timecodemodulation timecodemodulation;
		struct		timemode	timemode;
		struct		timefmt		timefmt;
		struct		ppo		ppo;
		struct		timecode	timecode;
		struct		propdelay	propdelay;
		struct		jamsync		jamsync;
		struct 		firmware	firmware;		/* firmware rev # */
		struct 		assembly	assembly;		/* flip the card or wonder */
		struct		model		model;			/* model type, 635/637 */
		struct		serial		serial;			/* serial number */
		struct		btfpctl		btfpctl;		/* driver control */
		struct		set_unixtime	set_unixtime;		/* set unixtime */
		/* XXX need biggest of data returned in ioctl calls, finish this */
		/* you are HERE XXX */
}__attribute__((packed));

/*
 * t0 and t1 are the machine time taken with clock_gettime() before and after latching time
 * into timereg registers. We can use this to get an approximate idea
 * of the measurement PCI latency without having having to worry with the overhead of the
 * system call.
 * Assuming that PCI write and PCI reads are the dominant factor, and assuming that there
 * is no arbitration, we can correlate btm latching with the idea of local time by
 * simply doing t0 + (t1 - t0) / 2. Further increase in precision can be done by better
 * adjusting for PCI access times. For best results, make sure the card is the only card on
 * its PCI bus, so to have deterministic times.
 */
struct btfp_ioctl_gettime {
	struct timespec t0;
	struct timespec time;
	struct timespec t1;
	uint32_t ns_precision;
	uint8_t locked; /* 1 if time is locked, 0 if freerunning */
	uint8_t timeoff; /* 1 if time offset > microsecond, 0 otherwise */
	uint8_t freqoff; /* 1 if freq offset > 10^-8, 0 otherwise */
};

/*
 * DPRAM is BAR1 based, 0x1000 bytes long.
 *
 * At the high end of DPRAM are three 2 byte offsets of interest:
 *
 * 0xffe-0xfff 	:	MSB, LSB of Input Area - usually 0x0102
 * 0xffc:0xffd	:	MSB, LSB of Output Area - usually 0x82
 * 0xffa:0xffb	:	MSB, LSB of GPS Packet Area - usually 0x02
 * 0xff8:0xff9	: 	MSB, LSB of Year Offset - usually 0, start of DPRAM.
 * 					Year will be bogus unless locked to something that
 *					populates the field - like GPS.
 *
 * NB: The access to the DPRAM is 8 bits wide, 1 byte. This implies and
 * requires bus_space_read_1() access. 
 */

/*
 * I really should set this dynamically after determining system HZ rate.
 * As a minor sop, blab will allow one to change this dynamically. 
 */
#define SNOOZER				20000 	
/*
 * TFP commands, arbitrary request numbers. These don't have explicit DPRAM
 * command codes associated.
 */
#define TFP_READ_TIME			5		/* NTP refclock i/f, do not renumber */
#define TFP_LATCH_TIMEREG		0x80	/* Latch time into time0/1 registers*/
#define TFP_LATCH_EVENTREG		0x81	/* Latch time into event0/1 registers*/
#define TFP_READ_EVENTREG		0x82	/* return contents of event0/1		*/
#define	TFP_SET_REG_CONTROL		0x83	/* manipulate control register		*/
#define TFP_FETCH_REG_CONTROL	0x84	/* get contents of control register	*/
#define TFP_UNLOCK_CAPTURE		0x85	/* reset capture lockout register	*/
#define	TFP_SET_REG_MASK		0x86	/* change interrupt mask register	*/
#define	TFP_FETCH_REG_MASK		0x87	/* read interrupt mask register		*/
#define TFP_SET_REG_INTSTAT 	0x88	/* change intstat register			*/
#define TFP_FETCH_REG_INTSTAT 	0x89	/* read interrupt status register	*/
#define TFP_BTFPCTL				0x8a	/* change level of output chatter	*/
#	define BLABBER_LESS			0		/* lower level */
#	define BLABBER_MORE			1		/* raise level */
#	define BLABBER_LEVEL		2		/* what is my current level ? */
#	define BLABBER_NONE			3		/* be silent */
#	define SNOOZER_FETCH		5
#	define SNOOZER_SET			6
/*
 * DPRAM command codes
 */
#define TFP_TIMEMODE		0x10 	/* set TFP timing mode					*/
#	define TIMEMODE_CODE	0x00	/* Irig A/B, IEEE 1344/NASA36			*/
#	define TIMEMODE_FREERUN	0x01	/* 10Mhz reference, int | ext			*/
#	define TIMEMODE_PPS		0x02	/* External 1 PPS signal				*/
#	define TIMEMODE_RTC		0x03	/* tfp onboard RTC IC. 				 	*/
#	define TIMEMODE_GPS		0x06	/* GPS. bc637 only.						*/

#define TFP_TIMEREG_FMT		0x11 	/* time register format					*/
#	define  BCD_TIME		0x00	/* Decimal								*/
#	define	UNIX_TIME		0x01	/* UNIX time - default					*/

#define TFP_MAJOR_TIME		0x12	/* set major time						*/ 
#define TFP_YEAR			0x13	/* set year								*/
#define TFP_PPO				0x14	/* periodic output						*/

#define TFP_INPUT_CODE_FMT	0x15	/* set input time code format			*/
#	define	IRIG_A			0x41	/* 'A' IRIG A format					*/
#	define	IRIG_B			0x42	/* 'B' IRIG B format, default for bc635	*/
#	define	IEEE_1344		0x49	/* 'I' IEEE 1344 format					*/
#	define	NASA36			0x4E	/* 'N' NASA36 format					*/

#define TFP_CODE_MODULATION	0x16	/* time code modulation					*/
#	define	AM_SINE			0x4d	/* 'M' amplitude modulated sine wave	*/
#	define	DCLS_PCM		0x44	/* 'D' DC level shift pulse code mod	*/

#define TFP_PROP_DELAY		0x17	/* propagation delay compensation		*/

#define TFP_UTC_INFO_CTRL	0x18	/* UTC from GPS receiver				*/
#  define  UTC_TIME			0x00	/* time from GPS is in UTC format		*/
#  define  GPS_TIME			0x01	/* time from GPS is GPS: UTC + leapsecs */
/*
 * TFP Data Request subcodes. Most correlate with command codes of the same
 * purpose.
 */
#define TFP_REQUEST_DATA	0x19	/* non-register TFP information			*/
/*		TFP_TIMEMODE		0x10	return current timemode input source	*/	
/*		TFP_TIMEREG_FMT		0x11	return current time register format		*/
/*		TFP_YEAR			0x13	return current year						*/
/*		TFP_PPO				0x14	return PPO parameters					*/
/*		TFP_INPUT_CODE_FMT	0x15	return current input timecode format	*/
/*		TFP_CODE_MODULATION 0x16	return current timecode modulation fmt	*/
/*		TFP_PROP_DELAY		0x17	return current propagation delay offset */
/*		TFP_UTC_INFO_CTRL	0x18	return UTC leapinfo, etc 637 only		*/
/* 		TFP_OUTPUT_CODE_FMT	0x1b	return current output timecode format	*/
/*		TFP_GEN_OFFSET		0x1c	return generator time offset 			*/
/*		TFP_LOCAL_TZ		0x1d	return local timezone offset & 1/2 hr	*/
/*		TFP_LEAP_SETTING	0x1e	return leap second event information	*/
/*		TFP_FIRM_VERSION	0x1f	return firmware version. XXX ? 			*/
/* 		TFP_CLOCK_SOURCE	0x20	return 'I' or 'E' oscillator used		*/
/* 		TFP_JAMSYNC_CTRL	0x21	return jamsync enablement status		*/
#define	TFP_BATTERY_STATUS	0x26	/* return battery status				*/
#	define	BATT_FAILED		0x00	/* bad battery							*/
#	define	BATT_OK			0x10	/* good battery. have a biscuit.		*/
/*		TFP_CLOCK_SLEW		0x29	request clock slew value, fly or free	*/
#define	TFP_LOCALTZ_OBS_QRY	0x41	/* return 0x?8 if local tz obs = yes	*/
#define TFP_REVISION		0x4f 	/* return PCI firmware revision number	*/
#define TFP_ASSEMBLY		0xf4  	/* return PCI assembly number			*/
#define TFP_HDW_FAB			0xf5	/* return Hardware fab part number		*/
#define TFP_MODEL			0xf6	/* return model number of tfp board		*/
#define TFP_SERIAL			0xfE	/* return serial number of tfp board	*/
/*
 * End of TFP Data Request subcodes. Continue with DPRAM command codes.
 */

#define	TFP_OUTPUT_CODE_FMT	0x1B	/* output timecode format 				*/
/*	define	IRIG_B			0x42	'B' IRIG B format						*/
/*	define	IEEE_1344		0x49	'I' IEEE 1344 format					*/

#define TFP_POR				0x1a	/* software reset						*/
#define TFP_GEN_OFFSET		0x1c	/* generator time offset				*/
#define TFP_LOCAL_TZ		0x1d	/* local time offset					*/
#define TFP_LEAP_SETTING	0x1e	/* leap second event setting			*/

#define TFP_CLOCK_SOURCE	0x20 	/* clock source, int/ext osc			*/
#	define	INTERNAL		0x49	/* 'I' use internal 10Mhz oscillator	*/	
#	define	EXTERNAL		0x45	/* 'E' use external 10Mhz oscillator	*/

#define TFP_JAMSYNC_CTRL	0x21 	/* jam sync 0/1 disable/enable 			*/
#define TFP_FORCE_JAMSYNC	0x22 	/* force jam sync						*/
#define TFP_OSCDISCP_CTRL	0x23	/* Oscillator Discipline control		*/
#define TFP_LOAD_DAC		0x24 	/* load DAC								*/
#define TFP_DISCIPLIN_GAIN	0x25	/* set disciplining gain				*/
#define TFP_SYNC_RTC		0x27	/* sync RTC to external time			*/
#define	TFP_BATT_DISCON		0x28	/* disconnect RTC battery for storage	*/
#define TFP_CLOCK_SLEW		0x29	/* slew clock value						*/
#define TFP_GPS_SENDPKT		0x30	/* send a packet to the GPS				*/
#define TFP_GPS_REQPKT		0x31	/* request common GPS packet			*/
#define TFP_GPS_MANUAL		0x32	/* manual GPS packet request			*/
#define TFP_GPS_TIMEFMT		0x33	/* set GPS time format					*/
/*  	UTC_TIME			0x00	time from GPS is in UTC format			*/
/*		GPS_TIME			0x01	time from GPS is GPS: UTC + leapsecs	*/
#define TFP_GPS_STATION_MODE	0x34 	/* GPS to station mode after lock 	*/
#	define	STATION			0x01
#	define	NO_STATION		0x00
#define TFP_OBS_LOCAL_TZ    0x40 	/* observe local time flag; see 0x1d	*/
#	define	LOCAL			0x01
#	define	NO_LOCAL		0x00
#define TFP_AUTO_YEAR_INCR  0x42 	/* year auto-increment flag				*/
#	define	AUTO_INCR		0x01
#	define	NO_AUTO_INCR	0x00
/*
 *	End of DPRAM command codes.
 */
/*
 * TFP device register defines
 */
#define TFP_REG_TIMEREQ	 		0x00	/* read to latch time into time0/1	*/
#define TFP_REG_EVENTREQ 		0x04	/* read to latch time into event0/1 */
#define TFP_REG_UNLOCK	 		0x08	/* read to reset capture lockout	*/

#define TFP_REG_CONTROL			0x10
/*
 * TFP_CR_LOCKEN:
 * Controls whether every event input signal causes a time capture, or only
 * the first. If lockout enabled, then the first event will cause the time to
 * be latched, and subsequent events will be ignored. Accessing the UNLOCK
 * register will enable the next event time to be captured.
 */
#	define TFP_CR_LOCKEN	 	0x01	/* Event capture lockout enable		*/
										/* =1 Enable lockout				*/
										/* =0 Disable lockout				*/

#	define TFP_CR_EVSOURCE	 	0x02	/* Event source select				*/
										/* =1 rising edge trigger from PPO 	*/	
										/* =0 event input triggered			*/

#	define TFP_CR_EVSENSE 	 	0x04	/* Event input edge select			*/
										/* =1 Rising edge active (falling on*/
										/*	12083 boards.					*/
									 	/* =0 Falling edge active (rising on*/ 
										/*	12083 boards.					*/	
/*
 * TFP_CR_EVENTEN:
 * Control external trigger ability to latch time into EVENT0/1 registers.
 * Disabling this will not stop interrupts from event input, use the interrupt
 * mask.  Select source of trigger with TFP_CR_EVSOURCE. 
 */
#	define TFP_CR_EVENTEN	 	0x08	/* Event capture register enable =1	*/
 

#	define TFP_CR_STREN			0x10	/* =1 enable strobe output 			*/
										/* =0 strobe output held low		*/
#	define TFP_CR_STRMODE	 	0x20	/* Time coincidence Strobe mode		*/
										/* =1 Use minor time only ->PPS out	*/
										/* =0 Use major & minor time 		*/
#	define TFP_CR_FREQSEL0	 	0x40	/* 2bit field.						*/
										/* 00 = 10Mhz.	01 = 5Mhz.			*/
										/* 1x = 1Mhz.						*/
#	define TFP_CR_FREQSEL1	 	0x80	/* hi bit of 2bit freq select		*/
#define	TFP_CR_RESERVED			0xffffff00	/* reserved bits 				*/

#define TFP_REG_ACK	 			0x14
										/* ACK register flags 				*/
#	define TFP_ACK_USRCMD    	0x01	/* TFP set - ACK user command		*/
										/* USER write - clear bit.			*/
#	define TFP_GPSPKT_RDY	 	0x04 	/* TFP set - GPS packet ready		*/
										/* bitset by TFP causes INT4.		*/
										/* USER write - clear bit.			*/
#	define TFP_MPUCMD_RDY	 	0x80 	/* USER set - command ready			*/
										/* for TFP fetch.					*/
/*
 * Interrupt processing registers. MASK bit lit allows that interrupt to be
 * propagated from TFP up to PCI bus. INTSTAT bit lit indicates the TFP
 * signalled that interrupt. Write bit lit to INTSTAT to clear interrupt.
 * Both registers share the same mapping, unmapped bits are undefined. 
 * On the 12083 board, logic levels of some external signals are available in 
 * INTSTAT.
 */
#define TFP_REG_MASK	 		0x18	/* bit dim disables that interrupt	*/
#define TFP_REG_INTSTAT	 		0x1c
#	define TFP_IR_EVENTINP	 	0x01	/* external event input occurred	*/
#	define TFP_IR_PERIODOUT 	0x02	/* periodic output					*/
#	define TFP_IR_STROBE	 	0x04  	/* time coincidence strobe			*/
#	define TFP_IR_1PPSOUT	 	0x08 	/* 1 PPS output						*/
#	define TFP_IR_GPSPKT	 	0x10  	/* GPS packet arrival from ACE iii 	*/
#	define TFP_IR_LL_EVENT		0x0100	/* logic level of event input		*/
#	define TFP_IR_LL_DCLS		0x0200	/* logic level of DCLS input		*/
#	define TFP_IR_LL_EXT1PPS	0x0400	/* logic level of External PPS inp	*/
/*	note that logic level signals do not generate PCI interrupts			*/
#	define TFP_IR_ALL			0x1f	/* all interrupts 					*/

#define TFP_REG_MINSTRB	 		0x20	/* strobe minor time register		*/
#define TFP_REG_MAJSTRB	 		0x24	/* strobe major time register		*/
#define TFP_REG_TIME0	 		0x30	/* minor time register 				*/
#define TFP_REG_TIME1	 		0x34	/* major time register  			*/
#define TFP_REG_EVENT0	 		0x38	/* event minor time register		*/
#define TFP_REG_EVENT1	 		0x3c	/* event major time	register		*/
/*
 * Minor time register (TIME0, EVENT0, MINSTRB) status bit flags 
 * TFP_MR_xxxxxxx
 */
#define TFP_MR_FLYWHEEL	 0x01000000		/* not locked to reference source	*/
#define TFP_MR_TIMEOFF	 0x02000000		/* time offset 0<|>1 X microsec		*/
/* 
 * X   5  for mode 0, else X   2
 */
#define TFP_MR_FREQOFF	 0x04000000		/* frequency offset 0<|>1 10-8		*/
#define TFP_WRITE_UNIX_TIME			253		/* write unix time */
#define TFP_READ_UNIX_TIME			254		/* read unix time */
/*
 * IOCTL call names for FreeBSD
 */
#define UBIO 					union btfp_ioctl_out
#define BTFP_READ_TIME			_IOR('1', TFP_READ_TIME, UBIO) 
#define BTFP_LATCH_TIMEREG		_IO('1', TFP_LATCH_TIMEREG)
#define BTFP_LATCH_EVENTREG		_IO('1', TFP_LATCH_EVENTREG)
#define BTFP_READ_EVENTREG		_IOR('1', TFP_READ_EVENTREG, UBIO)
#define BTFP_SET_REG_CONTROL	_IOW('1', TFP_SET_CONTROL, UBIO)
#define BTFP_FETCH_REG_CONTROL	_IOR('1', TFP_FETCH_REG_CONTROL, UBIO)
#define BTFP_UNLOCK_CAPTURE		_IO('1', TFP_UNLOCK_CAPTURE)		
#define	BTFP_SET_REG_MASK		_IOW('1', TFP_SET_REG_MASK, UBIO)
#define	BTFP_FETCH_REG_MASK		_IOR('1', TFP_FETCH_REG_MASK, UBIO)
#define BTFP_SET_REG_INTSTAT	_IOW('1', TFP_SET_REG_INTSTAT, UBIO)
#define BTFP_FETCH_REG_INTSTAT	_IOR('1', TFP_FETCH_REG_INTSTAT, UBIO)
#define BTFP_BTFPCTL			_IOWR('1', TFP_BTFPCTL, UBIO)
#define BTFP_TIMEMODE			_IOW('1', TFP_TIMEMODE, UBIO)
#define BTFP_TIMEREG_FMT		_IOW('1', TFP_TIMEREG_FMT, UBIO)
#define BTFP_MAJOR_TIME			_IOW('1', TFP_MAJOR_TIME, UBIO)
#define BTFP_YEAR				_IOW('1', TFP_YEAR, UBIO)
#define BTFP_PPO				_IOW('1', TFP_PPO, UBIO)
#define BTFP_INPUT_CODE_FMT		_IOW('1', TFP_INPUT_CODE_FMT, UBIO)
#define BTFP_CODE_MODULATION	_IOW('1', TFP_CODE_MODULATION, UBIO)
#define BTFP_PROP_DELAY			_IOW('1', TFP_PROP_DELAY, UBIO)
#define BTFP_UTC_INFO_CTRL		_IOW('1', TFP_UTC_INFO_CTRL, UBIO)
#define BTFP_REQUEST_DATA 		_IOWR('1', TFP_REQUEST_DATA, UBIO)
#define	BTFP_OUTPUT_CODE_FMT	_IOW('1', TFP_OUTPUT_CODE_FMT, UBIO)
#define BTFP_POR				_IO('1', TFP_POR)
#define BTFP_GEN_OFFSET			_IOW('1', TFP_GEN_OFFSET, UBIO)
#define BTFP_LOCAL_TZ			_IOW('1', TFP_LOCAL_TZ, UBIO)
#define BTFP_LEAP_SETTING		_IOW('1', TFP_LEAP_SETTING, UBIO)
#define BTFP_CLOCK_SOURCE		_IOW('1', TFP_CLOCK_SOURCE, UBIO)
#define BTFP_JAMSYNC_CTRL		_IOW('1', TFP_JAMSYNC_CTRL, UBIO)
#define BTFP_FORCE_JAMSYNC		_IO('1', TFP_FORCE_JAMSYNC)
#define BTFP_OSCDISCP_CTRL		_IOW('1', TFP_OSCDISCP_CTRL, UBIO)
#define BTFP_LOAD_DAC			_IOW('1', TFP_LOAD_DAC, UBIO)
#define BTFP_DISCIPLIN_GAIN		_IOW('1', TFP_DISCIPLIN_GAIN, UBIO)
#define BTFP_SYNC_RTC			_IO('1', TFP_SYNC_RTC)
#define	BTFP_BATT_DISCON		_IO('1', TFP_BATT_DISCON)
#define BTFP_CLOCK_SLEW			_IOW('1', TFP_CLOCK_SLEW, UBIO)
#define BTFP_READ_UNIX_TIME		_IOR('1', TFP_READ_UNIX_TIME, struct btfp_ioctl_gettime)
#define BTFP_WRITE_UNIX_TIME		_IOW('1', TFP_WRITE_UNIX_TIME, UBIO)
/*
 * That's all, folks!
 * End of btfp.h
 */
