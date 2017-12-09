BC635PCI  UNIX driver  for  FreeBSD 5.3




Contents :

1. btfp.h      -  Header file for the Symmetricom bc637-U PCI card, with GPS option


2. ACEiii.h    -  Header file for GPS data structures 
                  used to communicate with the BC637PCI-U onboard ACEiii GPS receiver
 
3. btfp.c      -  Driver Source file 
                  for Symmetricom bc637PCI-U GPS Synchronized, PCI Time & Frequency Processor

4. blab.c      -  Source for debugging code in the btfp0; 
                  controls device driver verbosity level & manipulates wait times

5. btfp_test.c -  Driver Source file
                  tests Symmetricom bc637PCI-U board functionality

6. gpst.c      -  Driver Source file
                  tests a variety of gps functions and displays the results

7. timeck.c    -  Driver Source file
                  verifies lock status  

8. btfp_setup.c - Driver Source file
                  sets up driver in UNIX_TIME, PPS mode and sets local time using the
                  new BTFP_WRITE_UNIX_TIME ioctl.
                  See btfp.c source for more explanation.

8. timeck_ux.c -  Driver Source file
                  uses the new BTFP_READ_UNIX_TIME ioctl for higher precision read of timestamp.
                  

8. refclock_bancomm.c 
               - clock driver for the  Datum/Bancomm bc635VME 
                modified to support Symmetricom bc637PCI-U Time & Frequency Processor
 		bus type (VME/VXI or PCI) and environment specified via the "mode" keyword ntp.conf server command
		server 127.127.16.u prefer mode m (...) 
 		Modes currently supported are 
 		  1		: FreeBSD PCI 635/637.
 		  2		: Linux or Windows PCI 635/637.
 		not specified, or other number: 
 				: Assumed to be VME/VXI legacy Bancomm card on Solaris.
 		Linux and Windows platforms require Symmetricoms' proprietary driver
 		for the TFP card. 
 		Tested on FreeBSD 5.3 with a 637 card. 

Installation :

untar into a directory and do a make, then make install to add the driver. 

Other targets in the makefile for the other cruft, should be pretty obvious.



This distribution includes a copy of refclock_bancomm.c, submitted to NTP project (bugzilla 451) 

The source includes configuration info. 

Check NTP site for distribution info.

Quick and dirty, build ntp with refclock 16 and then add

server 127.127.16.0  prefer mode 1 minpoll 4 maxpoll 4

to the ntp.conf for FreeBSD.


******


