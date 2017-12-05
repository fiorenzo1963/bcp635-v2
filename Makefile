SRCS=btfp.c bus_if.h device_if.h pci_if.h btfp.h ACEiii.h
KMOD=btfp

.include <bsd.kmod.mk>
gpst: gpst.c btfp.h ACEiii.h
	cc gpst.c -o gpst -Wall -lm
blab: blab.c btfp.h 
	cc blab.c -o blab -Wall 
ttfp: btfp_test.c btfp.h 
	cc btfp_test.c -o btfp_test -Wall 
timeck:	timeck.c btfp.h
	cc timeck.c -o timeck -Wall
install: btfp.ko
	cp btfp.ko /boot/kernel/btfp.ko
	kldunload btfp
	kldload btfp
