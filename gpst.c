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
 * Tests a variety of gps functions and displays the results.
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
#include "ACEiii.h"
/*
 * Prototypes
 */
int gps31_req(char subcmd, int fd, struct ACE_response *response);
int
gps32_req(int fd, struct ACE_prq_32 *request,
    struct ACE_response *response);
uint32_t gps_time(int fd);
uint32_t gps2ux(float tow, short week);
uint32_t io_options(int fd);
uint32_t machine_code(int fd);
uint32_t gps_message(int fd);
uint32_t last_computed_fix(int fd);
uint32_t last_raw_measurement(int fd);
uint32_t navigation_configuration(int fd);
uint32_t oscillator_offset(int fd);
uint32_t position_fix_f(int fd);
uint32_t receiver_health(int fd);
uint32_t satellite_ephemeris_status(int fd);
uint32_t satellite_health(int fd);
uint32_t satellite_selection(int fd);
uint32_t satellite_signal_status(int fd);
uint32_t satellite_tracking(int fd);
uint32_t software_version(int fd);
uint32_t UTC_parameters(int fd);
uint32_t velocity_XYZ_f(int fd);
uint32_t velocity_ENU_f(int fd);
void usage(void);
/*
 * Global variables
 */
struct ACE_response response;
struct ACE_prq_32 request;
const float pi = PI;
const double d_pi = PI;
const char callopts[] = "rh:s:f:t:o:mx";
int raw;
/*
 * Main program
 */
int
main(int argc, char **argv)
{
	int fd, havearg, rc;
	char ch;
	const char dname[] = "/dev/btfp0";

	fd = open(dname, O_RDWR);
	if (!(fd > 0)) {
		printf("tfp device %s open failed, rc is %x\n", dname, fd);
		return (fd);
	}
	raw = 0;
	havearg = 0;
	while ((ch = getopt(argc, argv, callopts)) != -1) {
		havearg = 1;
		switch (ch) {
		case 'h':	/* health */
			if (strcmp(optarg, "receiver") == 0 || *optarg == 'a') {
				receiver_health(fd);
				if (*optarg != 'a')
					break;
			}
			if (strcmp(optarg, "satellite") == 0 || *optarg == 'a') {
				satellite_health(fd);
				if (*optarg != 'a')
					break;
			}
			if (strcmp(optarg, "oscillator") == 0 || *optarg == 'a') {
				oscillator_offset(fd);
				break;
			}
			usage();
			break;
		case 'r':	/* raw mode toggle */
			raw = ~raw;
			break;
		case 's':	/* satellite info */
			if (strcmp(optarg, "track") == 0 || *optarg == 'a') {
				satellite_tracking(fd);
				if (*optarg != 'a')
					break;
			}
			if (strcmp(optarg, "ephemeris") == 0 || *optarg == 'a') {
				satellite_ephemeris_status(fd);
				if (*optarg != 'a')
					break;
			}
			if (strcmp(optarg, "message") == 0 || *optarg == 'a') {
				gps_message(fd);
				if (*optarg != 'a')
					break;
			}
			if (strcmp(optarg, "lastraw") == 0 || *optarg == 'a') {
				last_raw_measurement(fd);
				if (*optarg != 'a')
					break;
			}
			if (strcmp(optarg, "select") == 0 || *optarg == 'a') {
				satellite_selection(fd);
				if (*optarg != 'a')
					break;
			}
			if (strcmp(optarg, "signal") == 0 || *optarg == 'a') {
				if ((rc = satellite_signal_status(fd)) != 0) {
					printf("Satellite signal status failed %#x\n", rc);
					return (rc);
				}
				break;
			}
			usage();
			break;
		case 'f':	/* fix info */
			last_computed_fix(fd);

			if (strcmp(optarg, "position") == 0 || *optarg == 'a') {
				position_fix_f(fd);
				if (*optarg != 'a')
					break;
			}
			if (strcmp(optarg, "velocity") == 0 || *optarg == 'a') {
				velocity_XYZ_f(fd);
				velocity_ENU_f(fd);
				break;
			}
			usage();
			break;
		case 't':	/* time info */
			if (strcmp(optarg, "gps") == 0 || *optarg == 'a') {
				gps_time(fd);
				if (*optarg != 'a')
					break;
			}
			if (strcmp(optarg, "utc") == 0 || *optarg == 'a') {
				UTC_parameters(fd);
				break;
			}
			usage();
			break;
		case 'm':	/* misc stuff */
			software_version(fd);
			machine_code(fd);
			break;
		case 'o':	/* options */
			if (strcmp(optarg, "navigation") == 0 || *optarg == 'a') {
				navigation_configuration(fd);
				if (*optarg != 'a')
					break;
			}
			if (strcmp(optarg, "io") == 0 || *optarg == 'a') {
				io_options(fd);
				break;
			}
			usage();
			break;
		case 'x':	/* test stuff */
			break;
		case '?':
		default:
			usage();
			break;
		}
	}
	if (havearg == 0)
		usage();
	close(fd);
	return (0);
}				/* end main */
/*
 * Signal status, all satellites.
 */
uint32_t
satellite_signal_status(int fd)
{
	static int i;
	static struct rx47 *rx47_p;
	static union ieee754 fl;

	printf("Reading Satellite signal information, wait..\n");

	gps31_req(ACErpt_satellite_siglevels, fd, &response);
	rx47_p = (struct rx47 *)(&response.data);

	if (rx47_p->paircount > 8)
		return (-1);
	printf("%i satellite data pairs\n", rx47_p->paircount);

	for (i = 0; i < rx47_p->paircount; i++) {
		fl.l = be32toh(rx47_p->sigpair[i].siglevel);
		printf("PRN %2i siglevel %4.2f\n",
		    rx47_p->sigpair[i].satno, fl.f);
	}
	return (0);
}
/*
 * position fix from GPS via 0x31 command, XYZ and LLA single precision.
 */
uint32_t
position_fix_f(int fd)
{
	struct rx42 *rx42_p;
	uint32_t rc;
	struct rx4a20 *rx4a_p;
	float fpm = 3.2808;	/* feet per meter */
	union ieee754 tow, x, y, z, fixtime, clockbias;
	union ieee754 lat, longit, altit;

	if ((rc = gps31_req(ACErpt_position_fix_XYZ_f, fd, &response) != 0))
		return (rc);

	rx42_p = (struct rx42 *)(&response.data);

	tow.l = be32toh(rx42_p->time);
	printf("XYZ, position_s, @ GPS week + %.0f seconds\n", tow.f);

	x.l = be32toh(rx42_p->X);
	y.l = be32toh(rx42_p->Y);
	z.l = be32toh(rx42_p->Z);
	printf("X %-.0f, Y %-.0f, Z %-.0f\n", x.f, y.f, z.f);

	if ((rc = gps31_req(ACErpt_position_fix_LLA_f, fd, &response) != 0))
		return (rc);
	rx4a_p = (struct rx4a20 *)(&response.data);

	fixtime.l = be32toh(rx4a_p->fixtime);
	clockbias.l = be32toh(rx4a_p->clockbias);
	printf("LLA, position_s, @ GPS week + %.0f seconds, bias %.0f meters\n",
	    fixtime.f, clockbias.f);

	lat.l = be32toh(rx4a_p->latitude);
	longit.l = be32toh(rx4a_p->longitude);
	altit.l = be32toh(rx4a_p->altitude);
	printf("LLA latitude %-f, longitude %-f, altitude %-.2f meters, %-.2f ft\n",
	    lat.f, longit.f, altit.f, altit.f * fpm);

	float degr_lat = 180 / pi * lat.f;
	float degr_long = 180 / pi * longit.f;
	printf("LLA degrees latitude %f, longitude %f\n", degr_lat, degr_long);

	return (0);
}
uint32_t
satellite_selection(int fd)
/*
 * manual GPS command query
 */
{
	/* display satellites selected for fix determination */
	static int i, rc;
	static union ieee754 fl1, fl2, fl3, fl4;
	static uint8_t u8, svn;
	struct rx6d *rx6d_p;

	printf("Reading data for all-in-view satellite selection, wait, get coffee..\n");

	request.report_id = ACErpt_satellite_selection;
	request.report_len = 0;
	request.cmdpkt_id = ACEcmd_position_fix_mode;
	request.cmdpkt_len = 1;

	if ((rc = gps32_req(fd, &request, &response) != 0)) {
		printf("bad selection request, don't panic, try later\n");
		return (rc);
	}
	rx6d_p = (struct rx6d *)&response.data;

	u8 = rx6d_p->autodim;
	svn = (u8 & 0xf0) >> 4;

	printf("%i satellites providing %xD fix data: ", svn, (u8 & 0x07) - 1);
	for (i = 0; i < svn; i++)
		printf("%i ", rx6d_p->svprn[i]);
	printf("\n");
	fl1.l = be32toh(rx6d_p->pdop);
	fl2.l = be32toh(rx6d_p->hdop);
	fl3.l = be32toh(rx6d_p->vdop);
	fl4.l = be32toh(rx6d_p->tdop);
	printf("pdop %f, hdop %f, vdop %f, tdop %f\n", fl1.f, fl2.f, fl3.f, fl4.f);

	return (0);
}
/*
 * get the gps system message, which is ususally asciitrash, per usaf
 */
uint32_t
gps_message(int fd)
{
	static int i;
	static uint32_t rc;

	if ((rc = gps31_req(ACErpt_gps_message, fd, &response)) != 0)
		return (rc);

	printf("GPS system message, often encrypted by USAF->");

	for (i = 0; i < response.len - 1; i++)
		printf("%c", response.data[i]);
	printf("<-\n");

	return (0);
}
int
gps31_req(char subcmd, int fd, struct ACE_response *response)
{

	static char buf[255];
	int rc, i;

	bzero(response, sizeof(struct ACE_response));

	buf[0] = TFP_GPS_REQPKT;
	buf[1] = subcmd;

	rc = write(fd, buf, 2);
	if (rc < 0) {
		printf("bad write command %#x rc %#x\n", subcmd, rc);
		return (rc);
	}
	rc = read(fd, response, 128);
	if (rc < 0) {
		printf("bad read rc %#x \n", rc);
		return (rc);
	}
	if (raw != 0) {
		printf("raw data pktid %#x length %i\n", response->pid, response->len - 1);
		for (i = 0; i < response->len - 1; i++)
			printf("%2.2x ", response->data[i]);
		printf("\n");
	}
	return (0);
}
/*
 * Manual command 0x32 interface. Do not use with 0x31 subtypes.
 * Double or single format based on option settings.
 */
int
gps32_req(int fd, struct ACE_prq_32 *request, struct ACE_response *response)
{
	static int rc, i;
	static uint8_t j;

	bzero(response, sizeof(struct ACE_response));

	request->cid = TFP_GPS_MANUAL;
	if (raw != 0) {
		printf("cid %#x, cmdpkt_id %#x, cmdpkt_len %#x, rptid %#x, rptln %#x, ",
		    request->cid, request->cmdpkt_id, request->cmdpkt_len,
		    request->report_id, request->report_len);
		for (j = 0; j < 8; j++)
			printf("%.2x", request->data[j]);
		printf("\n");
	}
	rc = write(fd, request, sizeof(struct ACE_prq_32));
	if (rc < 0) {
		printf("bad write 0x32 pkt %#x, rc %#x, errno %#x \n",
		    request->cmdpkt_id, rc, errno);
		return (rc);
	}
	rc = read(fd, response, 128);
	if (rc < 0) {
		printf("bad read, try again later, pkt %#x, rc %#x, errno %#x\n",
		    request->cmdpkt_id, rc, errno);
		return (rc);
	}
	if (raw != 0) {
		printf("raw data pktid %#x length %i\n", response->pid,
		    response->len - 1);
		for (i = 0; i < response->len - 1; i++)
			printf("%2.2x ", response->data[i]);
		printf("\n");
	}
	return (0);
}
uint32_t
velocity_XYZ_f(int fd)
{
	static struct rx43 *rx43_p;
	static uint32_t rc;
	static union ieee754 fixtime, biasrate, x, y, z;

	if ((rc = gps31_req(ACErpt_velocity_fix_XYZ_f, fd, &response) != 0))
		return (rc);

	rx43_p = (struct rx43 *)(&response.data);

	fixtime.l = be32toh(rx43_p->fixtime);
	biasrate.l = be32toh(rx43_p->biasrate);
	printf("XYZ_s, @GPS week + %.0f seconds, bias %.0f m/s, ",
	    fixtime.f, biasrate.f);

	x.l = be32toh(rx43_p->X);
	y.l = be32toh(rx43_p->Y);
	z.l = be32toh(rx43_p->Z);
	printf("X %-.0f, Y %-.0f, Z %-.0f\n", x.f, y.f, z.f);
	return (0);
}
uint32_t
velocity_ENU_f(int fd)
{
	static struct rx56 *rx56_p;
	static uint32_t rc;
	static union ieee754 fixtime, biasrate, E, N, U;

	if ((rc = gps31_req(ACErpt_velocity_fix_ENU_f, fd, &response) != 0))
		return (rc);

	rx56_p = (struct rx56 *)(&response.data);

	fixtime.l = be32toh(rx56_p->fixtime);
	biasrate.l = be32toh(rx56_p->biasrate);
	printf("ENU_s, @GPS week + %.0f seconds, bias %.0f m/s, ",
	    fixtime.f, biasrate.f);

	E.l = be32toh(rx56_p->E);
	N.l = be32toh(rx56_p->N);
	U.l = be32toh(rx56_p->U);
	printf("E %-.0f, N %-.0f, U %-.0f\n", E.f, N.f, U.f);
	return (0);
}
uint32_t
gps_time(int fd)
{
	static struct rx41 *rx41_p;
	static uint32_t rc, ut;
	static union ieee754 tow, offset;
	static uint16_t week;
	time_t tm;

	if ((rc = gps31_req(ACErpt_GPS_time, fd, &response) != 0))
		return (rc);

	rx41_p = (struct rx41 *)(&response.data);
	tow.l = be32toh(rx41_p->GPStow);
	offset.l = be32toh(rx41_p->offset);
	week = be16toh(rx41_p->week);

	printf("%.0f GPS seconds into week #%i (%i), %.0f seconds from UTC.\n",
	    tow.f, week, week - 1023, offset.f);

	ut = gps2ux(tow.f, week);
	tm = ut;
	printf("GPS is %s", ctime(&tm));
	tm -= (int)offset.f;
	printf("UTC is %s", ctime(&tm));

	return (0);
}
/*
 * Get satellite health data. Zeros are good.
 */
uint32_t
satellite_health(int fd)
{
	static int i, badbirds;
	static struct rx49 *rx49_p;
	static uint32_t rc;

	if ((rc = gps31_req(ACErpt_almanac_health, fd, &response) != 0))
		return (rc);

	rx49_p = (struct rx49 *)(&response.data);
	badbirds = 0;

	for (i = 0; i < 32; i++)
		if (rx49_p->health[i] != 0x00)
			badbirds++;

	if (badbirds > 0) {
		printf("%i unhealthy satellites", badbirds);
		i = 0;
		while (badbirds > 0 && i < 32) {
			if (rx49_p->health[i] != 0x00) {
				printf(" %i", i + 1);
				i++;
				if (badbirds > 1) {
					printf(",");
					badbirds--;
				}
			} else
				i++;
		}
		printf("\n");
	} else
		printf("No sick satellites, mark your calendar!\n");

	return (0);
}
/*
 * ACEiii navigation and signal processor software version information
 */
uint32_t
software_version(int fd)
{
	static uint32_t rc;
	static struct rx45 *rx45_p;

	if ((rc = gps31_req(ACErpt_software_version, fd, &response) != 0))
		return (rc);

	rx45_p = (struct rx45 *)(&response.data);

	printf("Navigation processor version %i.%i, dated %i/%i/%i\n",
	    rx45_p->major_version, rx45_p->minor_version,
	    rx45_p->ver_month, rx45_p->ver_day, rx45_p->ver_year + 2000);

	printf("Signal processor version %i.%i, dated %i/%i/%i\n",
	    rx45_p->major_revision, rx45_p->minor_revision,
	    rx45_p->rev_month, rx45_p->rev_day, rx45_p->rev_year + 2000);

	return (0);
}
/*
 * Report on health of gps receiver
 */
uint32_t
receiver_health(int fd)
{
	static struct rx46 *rx46_p;
	static uint32_t rc;

	if ((rc = gps31_req(ACErpt_receiver_health, fd, &response) != 0))
		return (rc);

	rx46_p = (struct rx46 *)(&response.data);

	switch (rx46_p->status[0]) {
	case ACE_ok:
		printf("ACEiii receiver is OK, doing position fixes\n");
		break;
	case ACE_NOGPS_time:
		printf("Receiver does not have GPS time yet\n");
		break;
	case ACE_NEED_init:
		printf("Receiver needs initialization\n");
		break;
	case ACE_PDOP_high:
		printf("PDOP is too high\n");
		break;
	case ACE_NO_bird:
		printf("No usable satellites\n");
		break;
	case ACE_1_bird:
		printf("Only 1 usable satellite\n");
		break;
	case ACE_2_bird:
		printf("Only 2 usable satellites\n");
		break;
	case ACE_3_bird:
		printf("Only 3 usable satellites\n");
		break;
	case ACE_BAD_bird:
		printf("Selected satellite is unusable\n");
		break;
	default:
		printf("unexpected status code %#2.2x\n", rx46_p->status[0]);
		break;
	}
	if (rx46_p->status[1] & ACE_BBRAM_out) {
		printf("BRRAM not available at startup\n");
	}
	if (rx46_p->status[1] & ACE_ANT_fault) {
		printf("Antenna fault:");
		if (rx46_p->status[1] & ACE_ANT_short)
			printf(" short\n");
		else
			printf(" open\n");
	}
	return (0);
}
/*
 * Get machine code and additional status
 */
uint32_t
machine_code(fd)
{
	static struct rx4b *rx4b_p;
	static uint32_t rc;

	if ((rc = gps31_req(ACErpt_machine_code, fd, &response)) != 0)
		return (rc);

	rx4b_p = (struct rx4b *)(&response.data);

	printf("ACEiii machine id is %#2.2x\n", rx4b_p->machid);

	if (rx4b_p->status[0] & ACE_RTC_none) {
		printf("Real-Time Clock not available at power-up\n");
	} else
		printf("Real-Time Clock available at power-up\n");

	if (rx4b_p->status[0] & ACE_ALM_partial) {
		printf("Almanac is incomplete or not current\n");
	} else
		printf("Almanac is current\n");

	if (rx4b_p->status[1] & ACE_PKT_super)
		printf("Receiver supports Super packets\n");
	else
		printf("Receiver does not support Super packets\n");
	return (0);
}
uint32_t
oscillator_offset(fd)
{
	static uint32_t rc;
	static struct rx4d *rx4d_p;
	static union ieee754 offset;

	if ((rc = gps31_req(ACErpt_oscillator_offset, fd, &response) != 0))
		return (rc);

	rx4d_p = (struct rx4d *)(&response.data);

	offset.l = be32toh(rx4d_p->offset);

	printf("Oscillator offset is %f\n", offset.f);
	return (0);
}
uint32_t
UTC_parameters(int fd)
{
	static union ieee754 A0, A1, Tot;
	static uint32_t rc;
	static struct rx4f *rx4f_p;

	if ((rc = gps31_req(ACErpt_UTC_parameters, fd, &response) != 0))
		return (rc);

	rx4f_p = (struct rx4f *)(&response.data);
	A0.ll = be64toh(rx4f_p->A0);
	A1.l = be32toh(rx4f_p->A1);
	Tot.l = be32toh(rx4f_p->Tot);

	printf("A0 %g seconds, A1 %f sec/sec\n", A0.d, A1.f);

	printf("dTls %i seconds, dTlsf %i seconds, Tot %.0f seconds\n",
	    be16toh(rx4f_p->dTls), be16toh(rx4f_p->dTlsf), Tot.f);

	printf("WNt(UTC reference week #) %i, WNlsf(GPS week #) %i, DN(day #) %i\n", be16toh(rx4f_p->WNt), be16toh(rx4f_p->WNlsf), be16toh(rx4f_p->DN));

	return (0);
}
void
usage(void)
{
	printf("\ngpst displays information from the ACEiii gps on the bc637\n\n");
	printf("usage: gpst -[%s]\n", callopts);
	printf("\tr: toggle raw data dump on/off\n");
	printf("\th: health data\n");
	printf("\t\t -h [receiver | satellite | oscillator | a]\n");
	printf("\ts: satellite data\n");
	printf("\t\t -s [signal  | track | ephemeris | select | message | ...\n");
	printf("\t\t     lastraw | a]\n");
	printf("\tf: fix data\n");
	printf("\t\t -f [position | velocity | a]\n");
	printf("\tt: time data\n");
	printf("\t\t -t [gps | utc | a]\n");
	printf("\to: option data\n");
	printf("\t\t -o [navigation | io | a]\n");
	printf("\tm: misc data\n");
	printf("\tx: testing cruft, ymmv\n");
}
uint32_t
io_options(int fd)
{
/*
 * Display GPS receiver I/O options
 */
	static struct rx55 *rx55_p;
	static uint32_t rc;

	if ((rc = gps31_req(ACErpt_io_options, fd, &response) != 0))
		return (rc);

	rx55_p = (struct rx55 *)(&response.data);

	if (rx55_p->position & ACE_XYZ_out)
		printf("XYZ ECEF position output on, ");
	else
		printf("XYZ ECEF position output off, ");

	if (rx55_p->position & ACE_LLA_out)
		printf("LLA position output on, ");
	else
		printf("LLA position output off, ");

	if (rx55_p->position & ACE_LLALT_out)
		printf("LLA altitude MSA geoid\n");
	else
		printf("LLA altitude HAE (WGS-84)\n");

	if (rx55_p->position & ACE_ALT_in)
		printf("Altitude input MSL geoid, ");
	else
		printf("Altitude input HAE (WGS-84), ");

	if (rx55_p->position & ACE_PREC_pos)
		printf("Precision of position output is double float\n");
	else
		printf("Precision of position output is single float\n");

	if (rx55_p->position & ACE_SUPR_out)
		printf("Output all enabled Super Packets\n");
	else
		printf("Output no Super Packets\n");

	if (rx55_p->velocity & ACE_XYZ_vel)
		printf("XYZ ECEF velocity output on, ");
	else
		printf("XYZ ECEF velocity output off, ");

	if (rx55_p->velocity & ACE_ENU_vel)
		printf("ENU velocity output on, ");
	else
		printf("ENU velocity output off, ");

	if (rx55_p->timing & ACE_TIME_form)
		printf("Time format is UTC\n");
	else
		printf("Time format is GPS\n");

	if (rx55_p->range & ACE_RAW_aux)
		printf("Raw measurements on, ");
	else
		printf("Raw measurements off, ");

	if (rx55_p->range & ACE_RAW_5a)
		printf("Filtered PR's in 0x5A packets, ");
	else
		printf("Raw PR's in 0x5A packets, ");

	if (rx55_p->range & ACE_AMU_aux)
		printf("Output AMU's\n");
	else
		printf("Output dB Hz\n");

	return (0);
}
uint32_t
satellite_tracking(int fd)
{
/*
 * Satellite tracking status for channels 1-8;
 */
	static union ieee754 siglevel, elevation, azimuth;
	static struct rx5c *rx5c_p;
	static uint8_t channel, i;
	static uint32_t rc;

	for (i = 0; i < CHANNELS; i++) {
		bzero(&request, sizeof(request));

		request.report_id = ACErpt_satellite_tracking_status;
		request.report_len = 0;
		request.cmdpkt_id = ACEcmd_satellite_tracking_status;
		request.cmdpkt_len = 2;

		rc = gps32_req(fd, &request, &response);
		if (rc != 0)
			printf("bad rc iteration %i, continuing..\n", i);
		rx5c_p = (struct rx5c *)&response.data;

		siglevel.l = be32toh(rx5c_p->siglevel);
		elevation.l = be32toh(rx5c_p->elevation);
		azimuth.l = be32toh(rx5c_p->azimuth);
		channel = (rx5c_p->channel & 0x38) >> 3;

		printf("PRN %2i, chn %1i, eph %#x, siglevel %-2.2f",
		    rx5c_p->prn, channel, rx5c_p->ephemeris, siglevel.f);

		printf(", elevation %3.1f, aziumuth %3.1f\n",
		    180 / pi * elevation.f, 180 / pi * azimuth.f);
	}
	return (0);
}
uint32_t
satellite_ephemeris_status(int fd)
{
	static struct rx5b *rx5b_p;
	static uint8_t i;
	static union ieee754 toe, toc, ura;

	for (i = 0; i < 32; i++) {
		bzero(&response, sizeof(response));
		bzero(&request, sizeof(request));

		request.report_id = ACErpt_satellite_ephemeris_status;
		request.report_len = 0;
		request.cmdpkt_id = ACEcmd_satellite_ephemeris_status;
		request.cmdpkt_len = 2;
		/* something funky going on here - repeated calls give
		 * ascending prn numbers, which eventually wrap. Specifying
		 * the prn (or defaulting to zero) doesn't affect the results.
		 * Invoking the program again restarts at 1, not the last one
		 * done. Weird. the doc is silent, this looks like an oem
		 * thingy. */
		gps32_req(fd, &request, &response);
		rx5b_p = (struct rx5b *)&response.data;

		toe.l = be32toh(rx5b_p->toe);
		toc.l = be32toh(rx5b_p->toc);
		ura.l = be32toh(rx5b_p->URA);

		if (toe.f <= 0 || rx5b_p->fit != 0)
			continue;

		printf("PRN %i, Toe %.0f, Toc %.0f, dt %.0f, health %x, IODE %i, URA %.1f\n",
		    rx5b_p->prn, toe.f, toc.f, toe.f - toc.f, rx5b_p->health,
		    rx5b_p->IODE, ura.f);
	}
	return (0);
}
uint32_t
last_computed_fix(int fd)
{
	static int week;
	static uint32_t rc, tmp;
	static struct rx57 *rx57_p;
	static union ieee754 fixtime;
	time_t tm;

	if ((rc = gps31_req(ACErpt_last_computed_fix, fd, &response)) != 0)
		return (rc);

	rx57_p = (struct rx57 *)&response.data;
	if (rx57_p->source == 0x01)
		printf("Good current fix, ");
	else {
		printf("Currently no fix, try again later\n");
		return (-1);
	}

	printf("diag %#.2x, ", rx57_p->diag);

	fixtime.l = be32toh(rx57_p->fixtime);
	week = be16toh(rx57_p->week);

	tmp = gps2ux(fixtime.f, week);
	tm = tmp;
	printf("GPSweek %i + %6.0f s, %s", week, fixtime.f, ctime(&tm));
	return (0);
}
uint32_t
last_raw_measurement(int fd)
/*
 * raw satellite signal measurement information used in computing a fix.
 */
{
	static union ieee754 siglevel, code, doppler, time_om;
	static struct rx5a *rx5a_p;
	static uint8_t i;

	for (i = 0; i < CHANNELS; i++) {
		request.report_id = ACErpt_last_raw_measurement;
		request.report_len = 0;
		request.cmdpkt_id = ACEcmd_last_raw_measurement;
		request.cmdpkt_len = 2;

		gps32_req(fd, &request, &response);

		rx5a_p = (struct rx5a *)&response.data;
		siglevel.l = be32toh(rx5a_p->siglevel);
		doppler.l = be32toh(rx5a_p->doppler);
		code.l = be32toh(rx5a_p->code_phase);
		time_om.ll = be64toh(rx5a_p->time);
		printf("PRN %.2i, sgl %.1f, %f Hz, cphase %f, time %.0g\n",
		    rx5a_p->prn, siglevel.f, doppler.f, code.f, time_om.d);
	}

	return (0);
}
/*
 * Convert GPS time of week in float seconds to uint32_t unix epoch time.
 * The offset from GPS to UTC is not applied.
 */
uint32_t
gps2ux(float tow, short week)
{
	uint32_t uxtime;
	/*-
     * gyrate for UTC week start time difference from UNIX epoch
	 * Start of GPS week count is 00:00:00 1/6/1980.
	 * Start of UNIX epoch is     00:00:00 1/1/1970. Sigh.
	 */

	uxtime = (uint32_t) week + 522;
	uxtime = (uxtime * 604800) + (int)tow + (86400 * 3);
	return (uxtime);
}
/* packet BB, nav config query mode. */
uint32_t
navigation_configuration(int fd)
{
	int rc;
	struct rxbb *rxp;
	char *dim, *dgps, *ds;

	char *dimcode[] = {"Auto (2D/3D)", "Unexpected data", "Unexpected data",
	"Horizontal (2D)", "Full Position (3D)"};

	char *dgpsmode[] = {"DGPS off", "DGPS only", "DGPS auto",
	"Unexpected data"};

	char *dyn[] = {"Unexpected data", "Land", "Sea", "Air", "Stationary"};

	union ieee754 elevmask, dopmask, amumask, dopswitch;

	request.report_id = ACErpt_navigation_configuration;
	request.report_len = 0;
	request.cmdpkt_id = ACEcmd_navigation_configuration;
	request.cmdpkt_len = 2;
	request.data[0] = BB_QUERY;	/* query */

	if ((rc = gps32_req(fd, &request, &response)) != 0) {
		printf("navigation health call failed\n");
		return (rc);
	}
	rxp = (struct rxbb *)&response.data;

	switch (rxp->dim) {
	case 0:
		dim = dimcode[0];
		break;
	case 3:
		dim = dimcode[3];
		break;
	case 4:
		dim = dimcode[4];
		break;
	default:
		dim = dimcode[1];
		break;
	}
	switch (rxp->dgps) {
	case 0:
		dgps = dgpsmode[0];
		break;
	case 1:
		dgps = dgpsmode[1];
		break;
	case 2:
	case 3:
		dgps = dgpsmode[2];
		break;
	default:
		dgps = "Unexpected data";
	}
	switch (rxp->dynamics) {
	case 1:
		ds = dyn[1];
		break;
	case 2:
		ds = dyn[2];
		break;
	case 3:
		ds = dyn[3];
		break;
	case 4:
		ds = dyn[4];
		break;
	default:
		ds = dyn[0];
		break;
	}

	printf("%s, %s, %i sec, %s dynamic\n",
	    dim, dgps, rxp->dgps_limit, ds);
	elevmask.l = be32toh(rxp->elevation_mask);
	dopmask.l = be32toh(rxp->DOP_mask);
	amumask.l = be32toh(rxp->amu_mask);
	dopswitch.l = be32toh(rxp->DOP_switch);

	printf("Elevation mask %.4f, AMU mask %.1f ", elevmask.f, amumask.f);
	printf("DOP mask %.1f, DOP switch %.1f\n", dopmask.f, dopswitch.f);
	return (rc);
}
