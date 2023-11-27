// SPDX-License-Identifier: MIT
/*
 * read out positions from TI's AI2 GPS interface
 * should work with TI's /dev/tigps device
 * and also the patched mainline /dev/gnssX interface
 */
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <sys/select.h>
#include <time.h>
#ifndef NO_THREADS
#include <pthread.h>
#endif

/* we assume machine order = network order = le for simplity here */
struct __attribute__((__packed__)) measurement_sv {
	uint32_t fcount; /* probably ms since start of report */
	struct __attribute__((__packed__)) {
		uint8_t sv;
		uint16_t snr;
		uint16_t cno;
		uint8_t unknown[23];
	} svdata[];
};

struct __attribute__((__packed__)) position {
	uint32_t fcount; /* probably ms since start of report */
	uint16_t unknown1;
	int32_t lat;
	int32_t lon;
	int16_t altitude;
};

static bool nmeaout;

__attribute__((__format__ (__printf__, 1, 2)))
static void decode_info_out(const char *format, ...)
{
	va_list ap;
	if (nmeaout)
		return;

	va_start(ap, format);
	vfprintf(stdout, format, ap);
	va_end(ap);
}

__attribute__((__format__ (__printf__, 1, 2)))
static void decode_err_out(const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	vfprintf(nmeaout ? stderr : stdout, format, ap);
	va_end(ap);
}

static void out_nmealine(double lon, double lat,
			 double heading, double speed)
{
  char timestr[20];
  char datestr[20];
  char linebuf[100];
  time_t t; 
  struct tm tm;
  int chksum;
  int l,i;
  int lattdeg,longdeg;
  double lattmin, longmin;
  t = time(NULL);
  tm =* gmtime(&t);
  strftime(datestr, sizeof(datestr),"%d%m%y",&tm);
  strftime(timestr, sizeof(timestr),"%H%M%S.000", &tm);
  lattdeg = (int)lat;
  longdeg = (int)lon;
  lattmin = lat * 60.0 - lattdeg * 60.0;
  longmin = lon * 60.0 - longdeg * 60.0;
  snprintf(linebuf, sizeof(linebuf),"GPRMC,%s,A,%02d%07.4f,%c,%03d%07.4f,%c,%1f,%.1f,%s,,,A",
           timestr,
	   lattdeg, lattmin, lattdeg>0?'N':'S',
	   longdeg, longmin, longdeg>0?'E':'W',
	   speed / 1.852, heading < 0 ? (heading + 360) : heading,
	   datestr);

  l = strlen(linebuf);
  for(i = 0, chksum = 0; i < l;i++) {
    chksum ^= linebuf[i];
  }
  printf("$%s*%02X\r\n", linebuf, chksum & 0xff);
  fflush(stdout);
}

static void process_nmea(const uint8_t *data, int len)
{
	printf("nmea: %x %x %x %x:" , data[0], data[1], data[2], data[3]);
	if (len > 4) {
		fwrite(data + 4, 1, len - 4, stdout);
	}
}

static void process_position(const uint8_t *data, int len)
{
	const struct position *p = (const struct position *) data;
	int i;
	if (len < sizeof(struct position))
	       return;

	double lat = 90 *(double)p->lat / 2147483648.0;
	double lon = 180 *(double)p->lon / 2147483648.0;
	decode_info_out("position: fcount: %d, lat: %f lon: %f altitude: %.1f",
			p->fcount, lat, lon,
			(double)p->altitude / 2.0);
	data += sizeof(struct position);
	len -= sizeof(struct position);
	data += 15;
	len -= 15;
	len -= 13; /* unknown data at the end */
	decode_info_out("sv:");
	for(i = 0; i < len; i+= 6)
		decode_info_out(" %d", data[i]);
	decode_info_out("\n");

	if (nmeaout)
		out_nmealine(lon, lat, 0, 0);
}

static void process_measurement(const uint8_t *data, int len)
{
	int sats;
	int i;
	const struct measurement_sv *sv = (const struct measurement_sv *)data;
        if (len < 4)
	  return;

        sats = (len - 4) / sizeof(sv->svdata[0]);
	decode_info_out("measurement: fcount: %d, sats: %d\n", sv->fcount, sats);
	if ((len - 4) % sizeof(sv->svdata[0])) {
	    printf("measurement: excess data\n");
	}

	for(i = 0; i < sats; i++) {
		decode_info_out("SV: %d SNR: %.1f CNo: %.1f\n", sv->svdata[i].sv, (double)sv->svdata[i].snr / 10, (double)sv->svdata[i].cno / 10);
	}
}

static void process_packet(uint8_t type, const uint8_t *data, int len)
{
	decode_info_out("packet type %x, payload: %d\n", type, len);
	switch(type) {
	case 8:
		process_measurement(data, len);
		break;
	case 6: 
		process_position(data, len);
		break;
	case 0xd3:
		process_nmea(data, len);
		break;
	default:
		decode_info_out("unknown packet type %x len: %d\n", (int)type, len);
	}
}

static void write_init(int fd)
{
	uint8_t init1[] = {0x10,0x00,0xf5,0x01,0x00,0x01,0x07,0x01,0x10,0x03};
	uint8_t init2[] = {0x10,0x01,0xf1,0x01,0x00,0x05,0x08,0x01,0x10,0x03};
	uint8_t init3[] = {0x10,0x01,0xf0,0x00,0x00,0x01,0x01,0x10,0x03};
	uint8_t init4[] = {0x10,0x01,0x02,0x01,0x00,0x02,0x16,0x00,0x10,0x03};
	uint8_t init5[] = {0x10,0x01,0xed,0x01,0x00,0x00,0xff,0x00,0x10,0x03};
	uint8_t init6[] = {0x10,0x01,0x06,0x0d,0x00,0x01,0x0e,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x34,0x00,0x10,0x03};
	uint8_t init7[] = {0x10,0x01,0x02,0x01,0x00,0x03,0x17,0x00,0x10,0x03};

	write(fd, init1, sizeof(init1));
	usleep(200000);
	write(fd, init2, sizeof(init2));
	usleep(200000);
	write(fd, init3, sizeof(init3));
	usleep(200000);
	write(fd, init4, sizeof(init4));
	usleep(200000);
	write(fd, init5, sizeof(init5));
	usleep(200000);
	write(fd, init6, sizeof(init6));
	usleep(200000);
	write(fd, init7, sizeof(init7));
}


static void *read_loop(void *fdp)
{
	FILE *f;
	uint8_t gpsbuf[1024];
	int fd = *(int *)fdp;
	int c;
	int totalbufpos = 0;
	int bufpos;
	bool decoding_ack;
	uint16_t sum;
	int pktlen;
	f = fdopen(fd, "r+");
	bufpos = 0;
	sum = 0;
	pktlen = 0;
	while((c = fgetc(f)) != EOF) {
		totalbufpos++;
		if (bufpos == 0) {
			if (c != 0x10) {
				decode_err_out("d");
				continue;
			}
			sum = 0;	
			decoding_ack = false;
			decode_err_out("\n");
		}
		
		if (bufpos == 1) {
			if (c == 3) {
				decode_err_out("%04x unexpected end of packet\n", totalbufpos);
				bufpos = 0;
				continue;
			} else if (c == 2) {
				/* e.g. 100212001003 */
				decode_info_out("decoding ack\n");	
				decoding_ack = true;
			}
		}
		
		/* 0x10 needs to be unescaped */
		if ((c == 0x10) &&
		    (bufpos > 0) &&
		    (gpsbuf[bufpos-1] == 0x10)) {
			decode_err_out("unescaping 0x10\n");
			continue;
		}

		gpsbuf[bufpos] = c;
		bufpos++;
		if (bufpos <= 5 + pktlen) {
			sum += c;
		}
		if (bufpos == sizeof(gpsbuf)) {
			bufpos = 0;
			sum = 0;
			decode_err_out("overlong packet, throwing away\n");
		}
		if (bufpos == 5) {
			pktlen = gpsbuf[4];
			pktlen <<=8;
			pktlen |= gpsbuf[3];
		}
		
		if ((bufpos == 6) && (decoding_ack)) {
			bufpos = 0;
			sum = 0;
			decode_info_out("decoded ack\n");
			decoding_ack = false;
			continue;
		}

		if (bufpos == 5 + pktlen + 2) {
			uint16_t chk = gpsbuf[5 + pktlen + 1];
			chk <<= 8;
			chk |= gpsbuf[5 + pktlen];
			if (chk != sum)
				decode_err_out("%04x checksum mismatch %04x != %04x\n", totalbufpos, (int)chk, (int)sum);

			sum = 0;

		}
		if (bufpos == 5 + pktlen + 2 + 2) {
			bufpos = 0;

			if ((gpsbuf[5 + pktlen + 2] != 0x10) || (gpsbuf[5 + pktlen + 2 + 1] != 0x03)) {
				decode_err_out("%04x no terminator\n", totalbufpos);
			} /* else */ {
				decode_info_out("%04x report %0x received\n", totalbufpos, gpsbuf[2]);
				process_packet(gpsbuf[2], gpsbuf + 5, pktlen);
			}
			sum = 0;
		}
	}
	return NULL;
}

int main(int argc, char **argv)
{
	int fd;
	struct timeval tv;
	fd_set fds;
	if ((argc < 2) || !strcmp(argv[1], "--help")) {
		fprintf(stderr, "Usage: %s gnssdev [nmea]\n", argv[0]);
		return 1;
	}
	if ((argc > 2) && (!strcmp(argv[2], "nmea")))
		nmeaout = true;

        fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Cannot open %s\n", argv[1]);
		return 1;
	}

	fprintf(stderr, "Device opened, waiting 5s for input\n");
	FD_ZERO(&fds);
	FD_SET(fd, &fds);
	tv.tv_sec = 5;
	tv.tv_usec = 0;	
	if (select(fd + 1, &fds, NULL, NULL, &tv) <= 0) {
#ifndef NO_THREADS
		pthread_t thread;
		pthread_create(&thread, NULL, read_loop, &fd);
#endif
		fprintf(stderr, "no data, trying to init\n");
		write_init(fd);
#ifndef NO_THREADS
		pthread_join(thread, NULL);
		return 0;
#endif
	}
	read_loop(&fd);
}
