SRCS := btfp.c bus_if.h device_if.h pci_if.h btfp.h ACEiii.h
KMOD := btfp
APPS := blab btfp_setup btfp_test gpst timeck timeck_ux timeck_ux2 timeck_ux3 bcm635_shm

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
timeck_ux2: timeck_ux2.c btfp.h timespec_ops.h
	cc timeck_ux2.c -o timeck_ux2 -Wall
timeck_ux3: timeck_ux3.c btfp.h timespec_ops.h
	cc timeck_ux3.c -o timeck_ux3 -Wall
bcp635_shm: bcp635_shm.c btfp.h timespec_ops.h
	cc bcp635_shm.c -o bcp635_shm -Wall

apps: $(APPS)

#
# clean everything
#
_clean clean_ cleanup distclean:
	rm -f .depend.btfp.o *.o core *.core $(APPS)

install: btfp.ko
	cp btfp.ko /boot/kernel/btfp.ko
	-kldunload btfp
	kldload btfp
