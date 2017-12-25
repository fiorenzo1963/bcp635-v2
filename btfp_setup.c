/*-
 * Copyright (c) 2017 Fio Cattaneo	fio@cattaneo.us
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
#include <netinet/in.h>
/*#include <sys/param.h>*/
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "btfp.h"

int
main(int argc, char **argv)
{
	int i, fd;
	union btfp_ioctl_out	btfp;
	time_t t;

	const char *btfp_dev = "/dev/btfp0";

	if ((fd = open(btfp_dev, O_RDWR)) == -1) {
		printf("bad open of tfp device %s: %d (%s)\n", btfp_dev, fd, strerror(errno));
		return (fd);
	} else {
		printf("Open of tfp device %s: fd=%d\n", btfp_dev, fd);
	}

	bzero(&btfp, sizeof (btfp));
	btfp.timecodemodulation.id = TFP_CODE_MODULATION;
	btfp.timecodemodulation.modulation = DCLS_PCM; /* 'D' DC level shift */
	if ((i = ioctl(fd, BTFP_CODE_MODULATION, &btfp)) != 0) {
		printf(" ioctl set timecode modulation DC level failed rc = %x, errno = %d\n", i, errno);
		exit(2);
	}
	printf("timecode modulation DC level set\n");

	bzero(&btfp, sizeof (btfp));
	btfp.timemode.id = TFP_TIMEMODE;
	btfp.timemode.mode = TIMEMODE_PPS;
	if ((i = ioctl(fd, BTFP_TIMEMODE, &btfp)) != 0) {
		printf(" ioctl set timemode pps failed rc = %x, errno = %d\n", i, errno);
		exit(2);
	}
	printf("timemode pps set\n");

	bzero(&btfp, sizeof (btfp));
	btfp.timefmt.id = TFP_TIMEREG_FMT;
	btfp.timefmt.format = UNIX_TIME;
	if ((i = ioctl(fd, BTFP_TIMEREG_FMT, &btfp)) != 0) {
		printf(" ioctl set timereg_fmt failed rc = %x, errno = %d\n", i, errno);
		exit(2);
	}
	printf("timereg_fmt set\n");

	/*
	 * FIXME: add code to to wait until the begin of the second before setting this
	 * specs say that it should be set no more than 0.8 into the second
	 */

	bzero(&btfp, sizeof (btfp));
	time(&t);
	/*
	 * ioctl code will set the other fields
	 */
	btfp.set_unixtime.unix_time = (uint32_t)t;
	if ((i = ioctl(fd, BTFP_WRITE_UNIX_TIME, &btfp)) != 0) {
		printf(" ioctl set majortime failed rc = %x, errno = %d\n", i, errno);
		exit(2);
	}
	printf("majortime set to %u\n", btfp.set_unixtime.unix_time);

	return(0);
}
