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
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdlib.h>
#include <strings.h>
#include <syslog.h>
#include <time.h>
#include <sys/endian.h>
#include <unistd.h>

#include "btfp.h"

int flycheck(int fd, union btfp_ioctl_out *btm);

int
main(int argc, char **argv)
{
	static int i, fd;
	static uint32_t flywheel, time1;

	static union btfp_ioctl_out btm;
	const char *btfp_dev = "/dev/btfp0";
	struct tm majortm;

	if ((fd = open(btfp_dev, O_RDWR)) == -1) {
		printf("bad open of tfp device %s, fd is %d \n", btfp_dev, fd);
		return (fd);
	} else {
		printf("\nOpen of tfp device %s is good\n", btfp_dev);
		syslog(LOG_INFO,"btfp disruptive test program is starting\n");
		printf("This program causes loss of time lock for ~ 2 min\n");
	}
/*
 * Read time and determine if locked to source or flywheeling
 */
	flywheel = flycheck(fd, &btm);
/*
 * Display time input source - NB - this will cause loss of lock!!!!
*/ 
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_TIMEMODE;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf(" ioctl timemode failed rc = %x\n", i);
	else {
		printf("req %x timesource is ", btm.timemode.id);
		switch (btm.timemode.mode) {
		case TIMEMODE_CODE:
			printf("timecode\n");
			break;
		case TIMEMODE_FREERUN:
			printf("freerun\n");
			break;
		case TIMEMODE_PPS:
			printf("external PPS\n");
			break;
		case TIMEMODE_RTC:
			printf("tfp onboard RTC IC\n");
			break;
		case TIMEMODE_GPS:
			printf("GPS\n");
			break;
		default:
			printf(" XXX error in timesource\n");
			break;
		}
	}
/*
 * Display time register format
 */
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_TIMEREG_FMT;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf(" ioctl timefmt failed rc = %x\n", i);
	else {
		printf("req %x time register format is ", btm.timefmt.id);
		switch (btm.timefmt.format) {
		case BCD_TIME:
			printf("BCD\n");
			majortm.tm_yday = 0xffff0000 & time1;
			majortm.tm_hour = 0x0000ff00 & time1;
			majortm.tm_min = 0x000000f0 & time1;
			majortm.tm_sec = 0x0000000f & time1;
			printf("Local time is %s", asctime(&majortm));
			break;
		case UNIX_TIME:
			printf("UNIX\n");
			printf("Local time is %s", ctime((time_t *) (&time1)));
			break;
		default:
			printf(" XXX error in timefmt\n");
			break;
		}
	}
/*
 * Display Programmable Periodic Output parameters: PPO
 */
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_PPO;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf(" ioctl PPO req failed rc = %x\n", i);
	else {
		printf("req %x PPO parms are ", btm.ppo.id);
		printf("PPS sync is %x, Divider1 is %d, Divider2 is %d\n",
		    btm.ppo.sync, be16toh(btm.ppo.divider_1), be16toh(btm.ppo.divider_2));
	}
/*
 * Display input timecode format
 */
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_INPUT_CODE_FMT;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf(" ioctl input timecode format failed rc = %x\n", i);
	else {
		printf("req %x input timecode format is ", btm.timecode.id);
		switch (btm.timecode.timecode) {
		case IRIG_A:
			printf("IRIG-A\n");
			break;
		case IRIG_B:
			printf("IRIG-B\n");
			break;
		case IEEE_1344:
			printf("IEEE 1344\n");
			break;
		case NASA36:
			printf("NASA 36\n");
			break;
		default:
			printf("unspecified - error\n");
			break;
		}
	}
/*
 * Display input timecode modulation format
 */
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_CODE_MODULATION;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf(" ioctl input timecode modulation rc = %x\n", i);
	else {
		printf("req %x input timecode modulation is ", btm.timecode.id);
		switch (btm.timecode.timecode) {
		case AM_SINE:
			printf("AM SINE\n");
			break;
		case DCLS_PCM:
			printf("DC level shift PCM\n");
			break;
		default:
			printf("unspecified - error\n");
			break;
		}
	}
/*
 * Display propagation delay offset
 */
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_PROP_DELAY;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf(" ioctl propagation delay rc = %x\n", i);
	else {
		printf("req %x propagation delay is %d\n", btm.propdelay.id,
		    be32toh(btm.propdelay.propdelay));
	}
/*
 * Display output timecode format
 */
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_OUTPUT_CODE_FMT;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf(" ioctl output timecode format failed rc = %x\n", i);
	else {
		printf("req %x output timecode format is ", btm.timecode.id);
		switch (btm.timecode.timecode) {
		case IRIG_B:
			printf("IRIG-B\n");
			break;
		case IEEE_1344:
			printf("IEEE 1344\n");
			break;
		default:
			printf("unspecified - error\n");
			break;
		}
	}
/*
 * Display Jam-sync status
 */
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_JAMSYNC_CTRL;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf(" ioctl jamsync_ctrl failed rc = %x\n", i);
	else {
		printf("req %x jamsync enabled is ", btm.jamsync.id);
		printf("%x\n", btm.jamsync.ctrl);
	}
/*
 * Display firmware revision
 */
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_REVISION;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf(" ioctl firmware failed rc = %x\n", i);
	else {
		printf("req %x firmware revision is->", btm.firmware.id);
		for (i = 0; i < sizeof(btm.firmware.revision); i++)
			printf("%c", btm.firmware.revision[i]);
		printf("<-\n");
	}
/*
 * Display assembly part number (12043 or 12083, supposedly..)
 */
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_ASSEMBLY;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf("ioctl assembly failed rc = %x\n", i);
	else {
		printf("req %x assembly is %i\n", btm.assembly.id,
		    be16toh(btm.assembly.assembly));
	}
/*
 * Display hardware fab partno - gives me funky numbers....
 */
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_HDW_FAB;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf("ioctl hdw_fab failed rc = %x\n", i);
	else {
		printf("req %x hardware fab is %d\n", btm.assembly.id,
		    be16toh(btm.assembly.assembly));
	}
/*
 * Display model
 */
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_MODEL;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf(" ioctl model failed rc = %x\n", i);
	else {
		printf("req %x model is ", btm.model.id);
		for (i = 0; i < sizeof(btm.model.model); i++)
			printf("%c", btm.model.model[i]);
		printf("\n");
	}
/*
 * Display unit serial number
 */
	bzero(&btm, sizeof(btm));
	btm.inarea.cmd = TFP_REQUEST_DATA;
	btm.inarea.subcmd = TFP_SERIAL;

	if ((i = ioctl(fd, BTFP_REQUEST_DATA, &btm)) != 0)
		printf(" ioctl serial failed rc = %x\n", i);
	else {
		printf("req %x serial number is ", btm.serial.id);
		for (i = 0; i < sizeof(btm.serial.sn); i++)
			printf("%x", btm.serial.sn[i]);
		printf("\n");
	}
	close(fd);
	syslog(LOG_INFO,"btfp disruptive test program ended.\n");
	return(0);
}
int 
flycheck(int fd, union btfp_ioctl_out *btm)
{
	static int i, flywheel;

	bzero(btm, sizeof(union btfp_ioctl_out));

	if ((i = ioctl(fd, BTFP_READ_TIME, btm)) != 0) {
		printf(" ioctl failed rc = %x\n", i);
		return (i);
	} 

	flywheel = (btm->timereg.time0 & TFP_MR_FLYWHEEL) >> 24; 
	printf("time 1 is %#8.8x, time0 is %#8.8x\n", btm->timereg.time1,
			btm->timereg.time0);
	printf("Timesource status is ");
	if ( flywheel == 0) printf("locked\n"); 
		else printf("not locked %#8x\n", flywheel);
	return(flywheel);
}
