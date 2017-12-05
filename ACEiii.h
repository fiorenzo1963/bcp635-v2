
/*-
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
 * ACEiii.h
 * Header file for data structures used to communicate with the ACEiii GPS
 * receiver on the BC637PCI-U Time & Frequency Processor.
 * And yes, it's really an ACEiii and not an Acutime. Really.
 *
 * Documentation on the ACEiii can be found at http://www.trimble.com
 *
 * Anyone serious about doing GPS programming will require a copy of
 * the Interface Control Document, ICD200c.
 *
 * Get yours here: http://www.navcen.uscg.gov/pubs/gps/icd200/default.html
 * Chock full of cool, neat information about the guts of GPS. Enjoy.
 */
struct ACE_prq_32 {		/* packet request to ACE via TFP, cmd 0x32 */
	uint8_t cid;		/* 0x32 */
	uint8_t report_len;
	uint8_t report_id;
	uint8_t cmdpkt_len;
	uint8_t cmdpkt_id;
	uint8_t data[255];	/* request packet data		 */
}          __attribute__((packed));
/*
 * map the packets to be sent to the ACE III GPS.
 */
struct ACE_req {
	uint8_t cmd;		/* command id byte */
	uint8_t len;		/* inclusive length of pkt/data */
	uint8_t pid;		/* packet id */
	uint8_t data[255];	/* imperatives */
}       __attribute__((packed));
/*
 * ACE_response is used to map the packets received from the ACE III GPS.
 */
struct ACE_response {
	uint8_t len;		/* inclusive length of pkt/data */
	uint8_t pid;		/* id of packet from ACE III */
	uint8_t data[255];	/* payload - Report Packet */
}            __attribute__((packed));
#define 	PI									3.1415926535898
#define		CHANNELS							8
/*
 * Report packets and tfp 0x31 packet request types.
 */
#define		ACErpt_GPS_time						0x41
#define		ACErpt_position_fix_XYZ_f			0x42
#define		ACErpt_velocity_fix_XYZ_f			0x43
#define		ACErpt_software_version				0x45
#define		ACErpt_receiver_health				0x46
#define		ACErpt_satellite_siglevels			0x47
#define		ACErpt_gps_message					0x48
#define		ACErpt_almanac_health				0x49
#define		ACErpt_position_fix_LLA_f			0x4a
#define		ACErpt_machine_code					0x4b
#define		ACErpt_oscillator_offset			0x4d
#define		ACErpt_UTC_parameters				0x4f
#define		ACErpt_io_options					0x55
#define		ACErpt_velocity_fix_ENU_f			0x56
#define		ACErpt_last_computed_fix			0x57
#define		ACErpt_last_raw_measurement			0x5a
#define		ACErpt_satellite_ephemeris_status	0x5b
#define		ACErpt_satellite_tracking_status	0x5c
#define		ACErpt_satellite_selection			0x6d
#define		ACErpt_navigation_configuration		0xbb
/*
 * Command packets used with 0x32 tfp interface.
 */
#define		ACEcmd_position_fix_mode			0x24
#define 	ACEcmd_last_raw_measurement			0x3a
#define		ACEcmd_satellite_ephemeris_status	0x3b
#define		ACEcmd_satellite_tracking_status	0x3c
#define		ACEcmd_navigation_configuration		0xbb
/*
 * Packet maps - most need the __attribute__((packed)) to map correctly.
 * Data is represented by exact size format, uintnn_t, & does not reflect
 * the actual datatype. See the trimble ACEiii documentation for details.
 */
struct rx41 {			/* GPS time */
	uint32_t GPStow;	/* time of week, seconds */
	uint16_t week;		/* week number from jan 6, 1980 */
	uint32_t offset;	/* seconds offset between GPS and UTC */
}    __attribute__((packed));
struct rx42 {			/* single precision position fix */
	uint32_t X;
	uint32_t Y;
	uint32_t Z;
	uint32_t time;		/* seconds into the GPS week (sun mid) */
}    __attribute__((packed));
struct rx43 {			/* single precision velocity fix */
	uint32_t X;
	uint32_t Y;
	uint32_t Z;
	uint32_t biasrate;
	uint32_t fixtime;
}    __attribute__((packed));
struct rx45 {			/* Software version information */
	uint8_t major_version;	/* navigation processor */
	uint8_t minor_version;
	uint8_t ver_month;
	uint8_t ver_day;
	uint8_t ver_year;
	uint8_t major_revision;	/* signal processor */
	uint8_t minor_revision;
	uint8_t rev_month;
	uint8_t rev_day;
	uint8_t rev_year;
}    __attribute__((packed));
struct rx46 {			/* Health of Receiver */
	uint8_t status[2];
/*	Following defines are for status[1] only.  */
#define			ACE_ok			0x00	/* doing position fixes */
#define			ACE_NOGPS_time	0x01	/* don't have GPS time */
#define			ACE_NEED_init	0x02	/* Need initialization */
#define			ACE_PDOP_high	0x03	/* PDOP is too high   */
#define			ACE_NO_bird		0x08	/* no usable satellites */
#define			ACE_1_bird		0x09	/* only 1 usable
							 * satellite */
#define			ACE_2_bird		0x0a	/* only 2 good
							 * satellites */
#define			ACE_3_bird		0x0b	/* only 3 usable
							 * satellites */
#define			ACE_BAD_bird	0x0c	/* chosen satellite unusable */
/* Following defines are for status[2] only, bitsets.  */
#define				ACE_BBRAM_out	0x01	/* BBRAM @ startup: 0
							 * ok; 1 !ok */
#define				ACE_ANT_fault	0x10	/* Antenna fault if set */
#define				ACE_ANT_short 	0x20	/* Short if set, else
							 * open */
};
struct rx47 {			/* Signal Levels for all Satellites */
	uint8_t paircount;	/* Number of #/signal pairs in data */
	struct {
		uint8_t satno;
		uint32_t siglevel;
	}      __attribute__((packed)) sigpair[8];
}    __attribute__((packed));

struct rx48 {			/* GPS System Message */
	char msg[22];
};
struct rx49 {			/* Almanac Health page 25  */
	uint8_t health[32];	/* one each, sat 1-32 */
};
struct rx4a20 {			/* LLA GPS fix, 20 byte */
	uint32_t latitude;	/* radians; + for North, - South */
	uint32_t longitude;	/* radians; + for East, - West */
	uint32_t altitude;	/* meters (HAE or MSL) */
	uint32_t clockbias;	/* meters */
	uint32_t fixtime;	/* seconds (GPS or UTC) */
};
struct rx4a9 {			/* Reference Altitude */
	uint32_t altitude;	/* meters */
	uint32_t reserved;	/* reserved by ACE III */
	uint8_t altflag;	/* if set, RA is used, else not */
};
struct rx4b {			/* Mach/Code ID & Add'tl Status */
	uint8_t machid;		/* Receiver dependent */
	uint8_t status[2];	/* status bytes */
#define ACE_RTC_none		0x02	/* no RTC available */
#define ACE_ALM_partial	0x08	/* incomplete almanac */
#define ACE_PKT_super	0x01	/* super packets supported */
};
struct rx4d {			/* Oscillator Offset */
	uint32_t offset;	/* in Hz, varies */
};
struct rx4e {			/* Response to set GPS time */
	uint8_t reply;		/* ASCII 'Y' or 'N' */
};
struct rx4f {			/* UTC parameters ICD-200	 */
	uint64_t A0;		/* A0, see 2.3.3.5.2.4 & 	 */
	uint32_t A1;		/* a1, see 2.3.3.5.1.8 &    */
	uint16_t dTls;		/* delta T sub ls, tbl20-IX */
	uint32_t Tot;		/* T sub ot					 */
	uint16_t WNt;		/* WN sub t					 */
	uint16_t WNlsf;		/* WN sub lsf				 */
	uint16_t DN;		/* DN						 */
	uint16_t dTlsf;		/* delta T sub lsf			 */
}    __attribute__((packed));
struct rx55 {			/* I/O options */
	uint8_t position;	/* flag byte, position */
/*		These flags are the same for report packet 0x35	*/
#define				ACE_XYZ_out		0x01	/* XYZ ECEF output on =1 */
#define				ACE_LLA_out		0x02	/* LLA output on=1 */
#define				ACE_LLALT_out	0x04	/* MSL geoid=1, HAE=0 */
#define				ACE_ALT_in		0x08	/* MSL geoid=1, HAE=0 */
#define				ACE_PREC_pos	0x10	/* Single prec=0,
							 * double=1 */
#define				ACE_SUPR_out	0x20	/* =1 Super Packets
							 * output */
	uint8_t velocity;	/* flag byte, velocity */
#define				ACE_XYZ_vel		ACE_XYZ_out	/* see position */
#define				ACE_ENU_vel		0x02	/* 0=ENU off, 1=ENU on */
	uint8_t timing;		/* flag byte, timing */
#define				ACE_TIME_form	0x01	/* 0=GPS, 1=UTC */
	uint8_t range;		/* flag byte, range & aux */
#define				ACE_RAW_aux		0x01	/* 0=Raw off, 1=Raw on */
#define				ACE_RAW_5a		0x02	/* =1: Filtered PR's in
								 * 5a */
#define				ACE_AMU_aux		0x08	/* =0 dB Hz, =1 AMU */
}    __attribute__((packed));

struct rx56 {			/* Velocity fix, ENU */
	uint32_t E;		/* meters/sec, +E, -W */
	uint32_t N;		/* meters/sec, +N, -S */
	uint32_t U;		/* meters/sec, +Up, -Down */
	uint32_t biasrate;	/* meters/sec */
	uint32_t fixtime;	/* seconds, GPS or UTC */
}    __attribute__((packed));
struct rx57 {			/* Info re last computed fix */
	uint8_t source;		/* 00 no fix, 01 good fix */
	uint8_t diag;		/* Manufacturer diagnostic */
	uint32_t fixtime;	/* seconds, GPS time */
	uint16_t week;		/* weeks, GPS time */
}    __attribute__((packed));
struct rx58 {			/* Sat Sys Data Ack from Rcvr */
	uint8_t operation;	/* 1=Request data, 2=Load data */
	uint8_t type;		/* type of data */
#define				ACE_SSD_alm		0x02	/* Almanac */
#define				ACE_SSD_hlth	0x03	/* health */
#define				ACE_SSD_ion		0x04	/* Ionosphere */
#define				ACE_SSD_utc 	0x05	/* UTC */
#define				ACE_SSD_eph		0x06	/* Ephemeris; request
								 * only */
	uint8_t prn;		/* Satellite PRN number */
	uint8_t len;		/* length of data following */
	uint8_t data[255];	/* payload */
}    __attribute__((packed));
struct rx58_alm {		/* Almanac data */
	uint8_t t_oa_raw;	/* See sec 20.3.3.5.1.2 */
	uint8_t SV_HEALTH;	/* of ICD-GPS-200 */
	float e;		/* all angles are radians */
	float t_oa;
	float i_o;
	float OMEGADOT;
	float sqrt_a;
	float OMEGA_0;
	float omega;
	float M_0;
	float a_f0;
	float a_f1;
	float Axis;
	float n;
	float OMEGA_n;
	float ODOT_n;
	float t_zc;		/* (-1) if not avail */
	int16_t weeknum;
	int16_t wn_oa;
}        __attribute__((packed));
struct rx5a {			/* last raw measurement data */
	uint8_t prn;
	uint32_t reserved;
	uint32_t siglevel;	/* AMU or dBHz */
	uint32_t code_phase;	/* 1/16th chip */
	uint32_t doppler;	/* Hz */
	uint64_t time;		/* time of measurement */
}    __attribute__((packed));
struct rx5b {			/* satellite ephemeris status */
	uint8_t prn;
	uint32_t toc;		/* time of collection */
	uint8_t health;		/* flag */
	uint8_t IODE;
	uint32_t toe;		/* time of ephemeris */
	uint8_t fit;		/* fit interval flag */
	uint32_t URA;		/* User Range Accuracy */
}    __attribute__((packed));
struct rx5c {
/* Satellite tracking status */
	uint8_t prn;
	uint8_t channel;	/* bits 3-5 only */
	uint8_t acq;		/* flag */
	uint8_t ephemeris;	/* flag */
	uint32_t siglevel;
	uint32_t meas_time;	/* GPS time of last measurement */
	uint32_t elevation;	/* radians */
	uint32_t azimuth;	/* radians */
	uint32_t reserved;
}    __attribute__((packed));
/* Satellite selection data */
struct rx6d {
	uint8_t autodim;
	uint32_t pdop;
	uint32_t hdop;
	uint32_t vdop;
	uint32_t tdop;
	uint8_t svprn[8];	/* prn, number in bits 4-7 of autodim */
}    __attribute__((packed));
struct rxbb {
	uint8_t		subcode;
#	define		BB_QUERY				0
#	define		BB_COMMAND				3
	uint8_t		dim;
	uint8_t		dgps;
	uint8_t		dynamics;
	uint8_t		reserved;
	uint32_t	elevation_mask;
	uint32_t	amu_mask;
	uint32_t	DOP_mask;
	uint32_t	DOP_switch;
	uint8_t		dgps_limit;
	uint8_t		unused[18];
}	__attribute__((packed));
/*
 * Squink the float conversion. XXX watch this if converting to another
 * environment. Test the representation of a known value. Fair warning.
 */
union ieee754 {
	/* 0x480af840 should be decimal 142305    */
	float f;
	long l;
	double d;
	long long ll;
	uint32_t u32;
}       __attribute__((packed));
