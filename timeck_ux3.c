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
#include <string.h>
#include <time.h>
#include <sys/timex.h>

#include "btfp.h"

#include "timespec_ops.h"

int verbose = 0;

void btm_to_timespec(const union btfp_ioctl_out *btm, struct timespec *t_out)
{
	t_out->tv_sec = btm->timereg.time1;
	t_out->tv_nsec = (long)(btm->timereg.time0 & 0xfffff) * 1000L +
			 (long)((btm->timereg.time0 >> 20) & 0xf) * 100L;
}

/* converts scaled PPM drift to second per second drift */
#define FREQTOD(x)      (((double)(x)) / 65536e6) /* NTP to double */
/* converts scaled PPM to PPM drift */
#define FREQTOPPM(_x)	(FREQTOD(_x) * 1000.0 * 1000.0)

/* converts second per second drift to scaled PPM drift */
#define	DTOFREQ(x)	((int32)((x) * 65536e6)) /* double to NTP */
/* converts PPM drift to to scaled PPM drift */
#define PPMTOFREQ(_x)	DTOFREQ((_x) / 1000.0 * 1000.0)

double timex_to_us(struct timex *tx, long value)
{
	if (tx->status & STA_NANO)
		return value / 1000.0;
	else
		return value;
}

void print_timex_status(int status)
{
	printf("0x%x ", status);
	if (status & STA_PLL) printf("STA_PLL,");
	if (status & STA_PPSFREQ) printf("STA_PPSFREQ,");
	if (status & STA_PPSTIME) printf("STA_PPSTIME,");
	if (status & STA_FLL) printf("STA_FLL,");
	if (status & STA_INS) printf("STA_INS,");
	if (status & STA_DEL) printf("STA_DEL,");
	if (status & STA_UNSYNC) printf("STA_UNSYNC,");
	if (status & STA_FREQHOLD) printf("STA_FREQHOLD,");
	if (status & STA_PPSSIGNAL) printf("STA_PPSSIGNAL,");
	if (status & STA_PPSWANDER) printf("STA_PPSWANDER,");
	if (status & STA_PPSJITTER) printf("STA_PPSJITTER,");
	if (status & STA_PPSERROR) printf("STA_PPSERROR,");
	if (status & STA_CLOCKERR) printf("STA_CLOCKERR,");
	if (status & STA_NANO) printf("STA_NANO,"); else printf("STA_MICRO,");
	if (status & STA_MODE) printf("STA_MODE_PLL,"); else printf("STA_MODE_FLL,");
	if (status & STA_CLK) printf("STA_CLK_B"); else printf("STA_CLK_A");
}

void read_timex(void)
{
	struct timex timex;
	int time_status;

	memset(&timex, 0, sizeof (timex));
	timex.modes = 0; /* no modifications required */

	time_status = ntp_adjtime(&timex);

	if (verbose) printf("\n");
	if (verbose) printf("time_status = %d\n", time_status);

	if (verbose) printf("timex.modes     = 0x%x\n", timex.modes);		/* clock mode bits (wo) */
	if (verbose) printf("timex.offset    = %lf us\n", timex_to_us(&timex, timex.offset));		/* time offset (ns/us) (rw) */
	if (verbose) printf("timex.freq      = %lf ppm\n", FREQTOPPM(timex.freq));		/* frequency offset (scaled PPM) (rw) */
	if (verbose) printf("timex.maxerror  = %ld us\n", timex.maxerror);		/* maximum error (us) (rw) */
	if (verbose) printf("timex.esterror  = %ld us\n", timex.esterror);		/* estimated error (us) (rw) */
	if (verbose) printf("timex.status    = ");
	if (verbose) print_timex_status(timex.status);		/* clock status bits (rw) */
	if (verbose) printf("\n");
	if (verbose) printf("timex.constant  = %ld secs (poll interval)\n", timex.constant);		/* poll interval (log2 s) (rw) */
	if (verbose) printf("timex.precision = %lf us\n", timex_to_us(&timex, timex.precision));		/* clock precision (ns/us) (ro) */
	if (verbose) printf("timex.tolerance = %lf ppm\n", FREQTOPPM(timex.tolerance));		/* clock frequency tolerance (scaled PPM) (ro) */

	printf("%f,%f", timex_to_us(&timex, timex.offset), FREQTOPPM(timex.freq));
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
		printf("error: ioctl READ_UNIX_TIME failed rc = %x\n", ret);
		return;
	}
	clock_gettime(CLOCK_REALTIME, &tv_local1);
	clock_gettime(CLOCK_REALTIME, &tv_local2);

	if (verbose) printf(" tv0: %10ld.%09ld\n", tv_local0.tv_sec, tv_local0.tv_nsec);
	if (verbose) printf("\n");
	if (verbose) printf("  bt:  locked = %d\n", bt.locked);
	if (verbose) printf("  bt: timeoff = %d\n", bt.timeoff);
	if (verbose) printf("  bt: freqoff = %d\n", bt.freqoff);
	if (verbose) printf("  bt: ns_prec = %d\n", bt.ns_precision);
	if (verbose) printf(" bt0: %10ld.%09ld (local clock time t0)\n", bt.t0.tv_sec, bt.t0.tv_nsec);
	if (verbose) printf("  bt: %10ld.%09ld (HW board time)\n", bt.time.tv_sec, bt.time.tv_nsec);
	if (verbose) printf(" bt1: %10ld.%09ld (local clock time t1)\n", bt.t1.tv_sec, bt.t1.tv_nsec);
	if (timespec_gt(&bt.t0, &tv_local0) &&
			timespec_gt(&bt.t1, &bt.t0) &&
			timespec_gt(&tv_local1, &bt.t1) &&
			timespec_gt(&tv_local2, &tv_local1)) {
		struct timespec delta;
		timespec_sub(&delta, &bt.t1, &bt.t0);
		if (verbose) printf(" btd: %10ld.%09ld (bt1-bt0)\n", delta.tv_sec, delta.tv_nsec);
		if (delta.tv_sec != 0)
			printf("error: time delay too high: cannot calculate adjusted time\n");
		else {
			struct timespec delta1;
			long delta1_ns;
			delta.tv_nsec = delta.tv_nsec / 2;
			if (verbose) printf("hbtd: %10ld.%09ld (bt1-bt0)/2: (one way PCI bus delay)\n", delta.tv_sec, delta.tv_nsec);
			timespec_add(&bt0p, &bt.t0, &delta);
			if (verbose) printf("bt0p: %10ld.%09ld (bt0+hbtd): local clock adjusted to HW board time\n",
			       			bt0p.tv_sec, bt0p.tv_nsec);
			timespec_sub(&delta1, &bt.time, &bt0p);
			delta1_ns = timespec_to_ns(&delta1);
			if (delta1_ns == 0) {
				if (verbose) printf("   d: %ldns (bt-bt0p): hw_board_time.eq.local_clock (EXACT MATCH)\n", delta1_ns);
			} else if (delta1_ns < 0) {
				if (verbose) printf("   d: %ldns (bt-bt0p): hw_board_time.lt.local_clock (local_clock ahead)\n", delta1_ns);
			} else {
				if (verbose) printf("   d: %ldns (bt-bt0p): hw_board_time.gt.local_clock (local_clock behind)\n", delta1_ns);
			}

			printf("%ld.%09ld,%d%d%d,%ld.%09ld,%f,",
				bt0p.tv_sec, bt0p.tv_nsec,
				bt.locked, bt.timeoff, bt.freqoff,
				bt.time.tv_sec, bt.time.tv_nsec,
				(float)delta1_ns / 1000.0);
			read_timex();
			printf("\n");
		}
	} else {
		printf("error: timewarp present: cannot calculate adjusted time\n");
	}
	if (verbose) printf("\n");
	if (verbose) printf(" tv1: %10ld.%09ld\n", tv_local1.tv_sec, tv_local1.tv_nsec);
	if (verbose) printf(" tv2: %10ld.%09ld\n", tv_local2.tv_sec, tv_local2.tv_nsec);
	if (timespec_gt(&bt.t0, &tv_local0) &&
			timespec_gt(&bt.t1, &bt.t0) &&
			timespec_gt(&tv_local1, &bt.t1) &&
			timespec_gt(&tv_local2, &tv_local1)) {
		struct timespec delta;
		timespec_sub(&delta, &tv_local2, &tv_local1);
		if (verbose) printf("  kt: %10ld.%09ld (clock_gettime overhead)\n",
		       delta.tv_sec, delta.tv_nsec);
	} else {
		if (verbose) printf("error: timewarp present: cannot calculate adjusted time\n");
	}

	{
		struct timespec delta;
		timespec_sub(&delta, &tv_local1, &tv_local0);
		if (verbose) printf("  sc: %10ld.%09ld (system call duration)\n", delta.tv_sec, delta.tv_nsec);
	}

	/*
	 * sleep to
	 */
	{
		struct timespec tv;
		/*
		 * nanoseconds to sleep to get to 100ms before next second tick
		 */
		long us_next = (1000000000L - tv_local2.tv_nsec) / 1000L;
		us_next -= 100000L;
		usleep(us_next);
		for (;;) {
			clock_gettime(CLOCK_REALTIME, &tv);
			if (tv.tv_sec != tv_local2.tv_sec)
				break;
			usleep(10000L);
		}
	}
}

int
main(int argc, char **argv)
{
	int fd;
	int i;

	const char *btfp_dev = "/dev/btfp0";

	if ((fd = open(btfp_dev, O_RDWR)) == -1) {
		printf("bad open of tfp device %s, fd is %d \n", btfp_dev, fd);
		return (fd);
	}

	if (argc > 1 && strcmp(argv[1], "-v") == 0)
		verbose = 1;

	/*
	 * Read time and determine if locked to source or flywheeling
	 */
	for (i = 0; ; i++) {
		if (i % 20 == 0) {
			printf("# local_clock   flags(locked-timeoff-freqoff)   hw_board_clock   us_offset   us_ntpadjtime_offset   ntp_adjtime_ppm\n");
			printf("# us_offset = hw_board_time - local_clock\n");
			fflush(stdout);
		}
		read_unix_time(fd);
		fflush(stdout);
	}
	close(fd);
	return (0);
}
