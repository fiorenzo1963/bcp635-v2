all:
	make -f Makefile.kmod btfp.ko
	make -f Makefile.user all

clean:
	make -f Makefile.kmod clean
	make -f Makefile.user clean
