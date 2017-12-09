#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <sys/time.h>
#include <sys/endian.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <errno.h>
#include <stdatomic.h>

#include "btfp.h"

#include "timespec_ops.h"

#define	LEAP_NOWARNING	0x0	/* normal, no leap second warning */
#define	LEAP_ADDSECOND	0x1	/* last minute of day has 61 seconds */
#define	LEAP_DELSECOND	0x2	/* last minute of day has 59 seconds */
#define	LEAP_NOTINSYNC	0x3	/* overload, clock is free running */

struct shmTime {
	int    mode; /* 0 - if valid is set:
		      *       use values,
		      *       clear valid
		      * 1 - if valid is set:
		      *       if count before and after read of values is equal,
		      *         use values
		      *       clear valid
		      */
	volatile int    count;
	time_t		clockTimeStampSec;
	int		clockTimeStampUSec;
	time_t		receiveTimeStampSec;
	int		receiveTimeStampUSec;
	int		leap;
	int		precision;
	int		nsamples;
	volatile int    valid;
	unsigned	clockTimeStampNSec;	/* Unsigned ns timestamps */
	unsigned	receiveTimeStampNSec;	/* Unsigned ns timestamps */
	int		dummy[8];
};

struct shmTime *get_shmseg()
{
	struct shmTime *p;
	int shmid;

	/* 0x4e545030 is NTP0.
	 * Big units will give non-ascii but that's OK
	 * as long as everybody does it the same way.
	 */
	shmid = shmget(0x4e545030, sizeof (struct shmTime), IPC_CREAT | 0600);
	if (shmid == -1) { /* error */
		printf("SHM shmget (unit 0): %s\n", strerror(errno));
		return NULL;
	}
	p = (struct shmTime *)shmat(shmid, 0, 0);
	if (p == (struct shmTime *)-1) { /* error */
		printf("SHM shmat (unit 0): %s\n", strerror(errno));
		return NULL;
	}

	return p;
}

int read_unix_time(int fd, struct timespec *local_time, struct timespec *bt_time)
{
	struct btfp_ioctl_gettime bt;
	int ret;

	if ((ret = ioctl(fd, BTFP_READ_UNIX_TIME, &bt)) != 0) {
		printf("ioctl READ_UNIX_TIME failed: %s\n", strerror(errno));
		return -1;
	}

	printf("  bt:  locked = %d\n", bt.locked);
	printf("  bt: timeoff = %d\n", bt.timeoff);
	printf("  bt: freqoff = %d\n", bt.locked);
	printf("  bt: ns_prec = %d\n", bt.ns_precision);
	printf(" bt0: %ld.%09ld\n", bt.t0.tv_sec, bt.t0.tv_nsec);
	printf("  bt: %ld.%09ld\n", bt.time.tv_sec, bt.time.tv_nsec);
	printf(" bt1: %ld.%09ld\n", bt.t1.tv_sec, bt.t1.tv_nsec);

	if (bt.locked == 0) {
		printf("read_local_time: time not locked\n");
		return -1;
	}

	if (timespec_gt(&bt.t1, &bt.t0)) {
		struct timespec delta, delta1;
		timespec_sub(&delta, &bt.t1, &bt.t0);
		if (delta.tv_sec != 0) {
			printf("time delay too high: cannot calculate adjusted time\n");
			return -1;
		} else {
			struct timespec bt1a;
			delta.tv_nsec = delta.tv_nsec / 2;
			timespec_sub(&bt1a, &bt.t1, &delta);
			printf("bt1a: %ld.%09ld (bt1 local clock adjusted to bt reference -- %ld.%09ld one way PCI bus delay)\n",
			       bt1a.tv_sec, bt1a.tv_nsec,
			       delta.tv_sec, delta.tv_nsec);
			timespec_sub(&delta1, &bt1a, &bt.time);
			printf("   d: %ld.%09ld (bt1a - bt reference)\n",
			       delta1.tv_sec, delta1.tv_nsec);
			/**/
			*local_time = bt1a;
			*bt_time = bt.time;
			printf("read_unix_time: ok\n");
			return 0;
		}
	} else {
		printf("read_unix_time: timewarp present: cannot calculate adjusted time\n");
		return -1;
	}
}

int main(int argc, char **argv)
{
	struct shmTime *shm;
	int fd;
	int count;
	const char *btfp_dev = "/dev/btfp0";
	
	setbuf(stdout, NULL);
	setbuf(stderr, NULL);
	
	shm = get_shmseg();
	if (shm == NULL) {
		printf("cannot attach to SHM segment\n");
		exit(1);
	}

	printf("shm = %p\n", shm);

	if ((fd = open(btfp_dev, O_RDWR)) == -1) {
		printf("bad open of btfp device %s: %s\n", btfp_dev, strerror(errno));
		exit(1);
	} else {
		printf("opened btfp device %s\n", btfp_dev);
	}

	atomic_thread_fence(memory_order_seq_cst);
	shm->valid = 0;
	atomic_thread_fence(memory_order_seq_cst);
	memset(shm, 0, sizeof (struct shmTime));
	atomic_thread_fence(memory_order_seq_cst);
	shm->mode = 1;
	atomic_thread_fence(memory_order_seq_cst);

	for (count = 0; ; count++) {
		int ret;
		struct timespec local_time, bt_time;
		/*printf("loop:        ret = %d\n", ret);*/
		/*printf("loop: local_time = %ld.%09ld\n", local_time.tv_sec, local_time.tv_nsec);*/
		/*printf("loop:    bt_time = %ld.%09ld\n", bt_time.tv_sec, bt_time.tv_nsec);*/
#if 0
	int    mode; /* 0 - if valid is set:
		      *       use values,
		      *       clear valid
		      * 1 - if valid is set:
		      *       if count before and after read of values is equal,
		      *         use values
		      *       clear valid
		      */
#endif
		atomic_thread_fence(memory_order_seq_cst);
		while (shm->valid) {
			printf("SHM is still valid, not doing anything ****************************\n");
			atomic_thread_fence(memory_order_seq_cst);
			usleep(200 * 1000);
		}
		atomic_thread_fence(memory_order_seq_cst);

		ret = read_unix_time(fd, &local_time, &bt_time);

		shm->valid = 0;
		shm->count++;
		atomic_thread_fence(memory_order_seq_cst);
		if (ret == -1) {
			printf("loop: ret = %d: LEAP_NOTINSYNC\n", ret);
			shm->leap = LEAP_NOTINSYNC;
			shm->precision = (-18);
			atomic_thread_fence(memory_order_seq_cst);
			shm->count++;
			shm->valid = 1;
			atomic_thread_fence(memory_order_seq_cst);
		} else {
			printf("loop: ret = %d: LEAP_NOWARNING\n", ret);
			shm->clockTimeStampSec = (time_t)bt_time.tv_sec;
			shm->clockTimeStampUSec = (int)(bt_time.tv_nsec / 1000);
			shm->clockTimeStampNSec = (unsigned)bt_time.tv_nsec;
			shm->receiveTimeStampSec = (time_t)local_time.tv_sec;
			shm->receiveTimeStampUSec = (int)(local_time.tv_nsec / 1000);
			shm->receiveTimeStampNSec = (unsigned)local_time.tv_nsec;
			shm->leap = LEAP_NOWARNING;
			shm->precision = (-21); /* about 50 ns */
			atomic_thread_fence(memory_order_seq_cst);
			shm->count++;
			shm->valid = 1;
			atomic_thread_fence(memory_order_seq_cst);
		}
		atomic_thread_fence(memory_order_seq_cst);
	}
}
