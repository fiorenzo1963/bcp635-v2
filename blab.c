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
 * blab controls the verbosity level of debugging code in the btfp0
 * device driver. It also manipulates wait times.
 */
#include <sys/ioctl.h>
#include <sys/param.h>
#include <sys/endian.h>
#include <sys/uio.h>

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>

#include "btfp.h"

void usage(void);
uint32_t print_regs(int fd);
const char callopts[] = "lmqsz:";
const char dname[] = "/dev/btfp0";

int
main(int argc, char **argv)
{
	static int fd, i, havearg;
	static char ch;
	static union btfp_ioctl_out btm;

	fd = open(dname, O_RDWR);

	if (!(fd > 0)) {
		printf("Bad rc %#x opening tfp device %s\n", fd, dname);
		return (fd);
	}
	havearg = 0;

	while ((ch = getopt(argc, argv, callopts)) != -1) {
		havearg = 1;
		switch (ch) {
		case 'l':
			btm.btfpctl.id = BLABBER_LESS;
			if ((i = ioctl(fd, BTFP_BTFPCTL, &btm)) != 0) {
				printf("ioctl failed\n");
				return (i);
			}
			break;
		case 'm':
			btm.btfpctl.id = BLABBER_MORE;
			if ((i = ioctl(fd, BTFP_BTFPCTL, &btm)) != 0) {
				printf("ioctl failed\n");
				return (i);
			}
			break;
		case 'q':
			btm.btfpctl.id = BLABBER_NONE;
			if ((i = ioctl(fd, BTFP_BTFPCTL, &btm)) != 0) {
				printf("ioctl failed\n");
				return (i);
			}
			break;
		case 's':				/* show things */
			print_regs(fd);
			btm.btfpctl.id = BLABBER_LEVEL;
			if ((i = ioctl(fd, BTFP_BTFPCTL, &btm)) != 0) {
				printf("ioctl failed\n");
				return (i);
			} else
				printf("blabber level is %#x \n", btm.btfpctl.u32);

			btm.btfpctl.id = SNOOZER_FETCH;
			if ((i = ioctl(fd, BTFP_BTFPCTL, &btm)) != 0) {
				printf("snoozer fetch ioctl failed %#x\n", i);
				return (i);
			} else
				printf("snoozer value is %#x %i\n", 
					btm.btfpctl.u32, btm.btfpctl.u32);
			break;
		case 'z':
			if ((btm.btfpctl.u32 = (uint32_t) strtol(optarg, NULL, 10)) == 0) {
				printf("bad arg %s failed conversion to long\n", optarg);
				break;
			}
			btm.btfpctl.id = SNOOZER_SET;
			if ((i = ioctl(fd, BTFP_BTFPCTL, &btm)) != 0) {
				printf("snoozer set ioctl failed %#x\n", i);
				return (i);
			}
			printf("Snooze value set to %i %#x\n", btm.btfpctl.u32, 
					btm.btfpctl.u32);
			break;
		case '?':
		default:
			usage();
			break;
		}
	}
	if (havearg == 0)
		usage();
	return (0);
}				/* end main */
void
usage(void)
{
	printf("\nblab manipulates %s driver values\n", dname);
	printf("\nusage: blab [-%s]\n", callopts);
	printf("\t m: more blabber\n");
	printf("\t l: less blabber\n");
	printf("\t s: show blabber level & snooze time\n");
	printf("\t q: no blabber\n");
	printf("\t z int: set snooze time to int\n");
}
uint32_t
print_regs(int fd) 
{
	int i;
	union btfp_ioctl_out btm;
	uint32_t intstat, mask, ctrl;

	if ((i = ioctl(fd, BTFP_FETCH_REG_INTSTAT , &btm)) != 0) {
		printf("fetch inststat ioctl failed %#x\n", i);
				return (i);
	}
	intstat = btm.intstat;
	if ((i = ioctl(fd, BTFP_FETCH_REG_CONTROL , &btm)) != 0) {
		printf("fetch control reg ioctl failed %#x\n", i);
				return (i);
	}	
	ctrl = btm.ctrlreg;
	if ((i = ioctl(fd, BTFP_FETCH_REG_MASK , &btm)) != 0) {
		printf("fetch mask reg ioctl failed %#x\n", i);
				return (i);
	}
	mask = btm.intmask;
	printf("inststat %#8.8x, control %#8.8x, mask %#8.8x\n",
			intstat, ctrl, mask);
	return(0);
}	
