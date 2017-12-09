SRCS=btfp.c bus_if.h device_if.h pci_if.h btfp.h ACEiii.h
KMOD=btfp

#
# default target will build both the kernel driver and
# the sample apps.
#
all:
	make btfp.ko
	make apps

.include <bsd.kmod.mk>

gpst: gpst.c btfp.h ACEiii.h
	cc gpst.c -o gpst -Wall -lm
blab: blab.c btfp.h 
	cc blab.c -o blab -Wall 
btfp_test: btfp_test.c btfp.h
	cc btfp_test.c -o btfp_test -Wall 
btfp_setup: btfp_setup.c btfp.h
	cc btfp_setup.c -o btfp_setup -Wall 
timeck:	timeck.c btfp.h timespec_ops.h
	cc timeck.c -o timeck -Wall
timeck_ux: timeck_ux.c btfp.h timespec_ops.h
	cc timeck_ux.c -o timeck_ux -Wall
bcp635_shm: timeck_ux.c btfp.h timespec_ops.h
	cc bcp635_shm.c -o bcp635_shm -Wall

apps: gpst blab btfp_test timeck timeck_ux btfp_setup bcp635_shm

#
# clean everything
#
_clean clean_ cleanup distclean:
	make clean
	make clean_apps
	rm -f .depend.btfp.o

#
# clean apps
#
clean_apps:
	rm -f *.o core *.core
	rm -f gpst blab timeck btfp_test btfp_setup timeck_ux bcp635_shm

install: btfp.ko
	cp btfp.ko /boot/kernel/btfp.ko
	-kldunload btfp
	kldload btfp
