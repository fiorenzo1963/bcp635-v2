#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdlib.h>
#include <strings.h>
#include <time.h>
#include <sys/time.h>
#include <sys/endian.h>
#include <unistd.h>

#include "btfp.h"

#include "timespec_ops.h"

void flycheck(int fd);
void read_unix_time(int fd);

int
main(int argc, char **argv)
{
	int fd;

	const char *btfp_dev = "/dev/btfp0";

	if ((fd = open(btfp_dev, O_RDWR)) == -1) {
		printf("bad open of tfp device %s, fd is %d \n", btfp_dev, fd);
		return (fd);
	}
	/*
	 * Read time and determine if locked to source or flywheeling
	 */
	read_unix_time(fd);
	close(fd);
	return (0);
}

void btm_to_timespec(const union btfp_ioctl_out *btm, struct timespec *t_out)
{
	t_out->tv_sec = btm->timereg.time1;
	t_out->tv_nsec = (long)(btm->timereg.time0 & 0xfffff) * 1000L +
			 (long)((btm->timereg.time0 >> 20) & 0xf) * 100L;
}

void read_unix_time(int fd)
{
	struct btfp_ioctl_gettime bt;
	int ret;
	struct timespec tv_local0, tv_local1, tv_local2;
	struct timespec bt0p;

	bzero(&bt, sizeof (bt));

	clock_gettime(CLOCK_REALTIME, &tv_local0);
	if ((ret = ioctl(fd, BTFP_READ_UNIX_TIME, &bt)) != 0) {
		printf("ioctl READ_UNIX_TIME failed rc = %x\n", ret);
		return;
	}
	clock_gettime(CLOCK_REALTIME, &tv_local1);
	clock_gettime(CLOCK_REALTIME, &tv_local2);

	printf(" tv0: %10ld.%09ld\n", tv_local0.tv_sec, tv_local0.tv_nsec);
	printf("\n");
	printf("  bt:  locked = %d\n", bt.locked);
	printf("  bt: timeoff = %d\n", bt.timeoff);
	printf("  bt: freqoff = %d\n", bt.locked);
	printf("  bt: ns_prec = %d\n", bt.ns_precision);
	printf(" bt0: %10ld.%09ld (local clock time t0)\n", bt.t0.tv_sec, bt.t0.tv_nsec);
	printf("  bt: %10ld.%09ld (HW board time)\n", bt.time.tv_sec, bt.time.tv_nsec);
	printf(" bt1: %10ld.%09ld (local clock time t1)\n", bt.t1.tv_sec, bt.t1.tv_nsec);
	if (timespec_gt(&bt.t0, &tv_local0) &&
			timespec_gt(&bt.t1, &bt.t0) &&
			timespec_gt(&tv_local1, &bt.t1) &&
			timespec_gt(&tv_local2, &tv_local1)) {
		struct timespec delta, delta1;
		timespec_sub(&delta, &bt.t1, &bt.t0);
		printf(" btd: %10ld.%09ld (bt1-bt0)\n", delta.tv_sec, delta.tv_nsec);
		if (delta.tv_sec != 0)
			printf("time delay too high: cannot calculate adjusted time\n");
		else {
			long d_ns = 0;
			delta.tv_nsec = delta.tv_nsec / 2;
			printf("hbtd: %10ld.%09ld (bt1-bt0)/2: (one way PCI bus delay)\n", delta.tv_sec, delta.tv_nsec);
			timespec_add(&bt0p, &bt.t0, &delta);
			printf("bt0p: %10ld.%09ld (bt0+hbtd): local clock adjusted to HW board time\n",
			       bt0p.tv_sec, bt0p.tv_nsec);
			timespec_sub(&delta1, &bt0p, &bt.time);
			d_ns = timespec_to_ns(&delta1);
			if (d_ns == 0)
				printf("   d: %ldns (bt0p-bt): locak_clock-hw_board_time (EXACT MATCH)\n", d_ns);
			else if (d_ns < 0)
				printf("   d: %ldns (bt0p-bt): locak_clock-hw_board_time (local_clock behind)\n", d_ns);
			else
				printf("   d: %ldns (bt0p-bt): locak_clock-hw_board_time (local_clock ahead)\n", d_ns);

		}
	} else {
		printf("timewarp present: cannot calculate adjusted time\n");
	}
	printf("\n");
	printf(" tv1: %10ld.%09ld\n", tv_local1.tv_sec, tv_local1.tv_nsec);
	printf(" tv2: %10ld.%09ld\n", tv_local2.tv_sec, tv_local2.tv_nsec);
	if (timespec_gt(&bt.t0, &tv_local0) &&
			timespec_gt(&bt.t1, &bt.t0) &&
			timespec_gt(&tv_local1, &bt.t1) &&
			timespec_gt(&tv_local2, &tv_local1)) {
		struct timespec delta;
		timespec_sub(&delta, &tv_local2, &tv_local1);
		printf("  kt: %10ld.%09ld (clock_gettime overhead)\n",
		       delta.tv_sec, delta.tv_nsec);
	} else {
		printf("timewarp present: cannot calculate adjusted time\n");
	}

	{
		struct timespec delta;
		timespec_sub(&delta, &tv_local1, &tv_local0);
		printf("  sc: %10ld.%09ld (system call duration)\n", delta.tv_sec, delta.tv_nsec);
	}
}
