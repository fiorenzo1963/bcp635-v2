#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdlib.h>
#include <strings.h>
#include <time.h>
#include <sys/endian.h>
#include <unistd.h>

#include "btfp.h"

int flycheck(int fd, union btfp_ioctl_out *btm);

int
main(int argc, char **argv)
{
	static int i, fd;
	static uint32_t flywheel;

	static union btfp_ioctl_out btm;
	const char *btfp_dev = "/dev/btfp0";
	struct tm majortm;

	if ((fd = open(btfp_dev, O_RDWR)) == -1) {
		printf("bad open of tfp device %s, fd is %d \n", btfp_dev, fd);
		return (fd);
	}
/*
 * Read time and determine if locked to source or flywheeling
 */
	flywheel = flycheck(fd, &btm);
	close(fd);
	return (0);
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

	printf("Timesource status is ");

	if (flywheel == 0)
		printf("locked ");
	else
		printf("not locked %#8x \n", flywheel);

	printf("%s", ctime(&(btm->timereg.time1)));

	return (flywheel);
}
