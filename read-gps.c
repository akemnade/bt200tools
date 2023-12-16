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
#include <stddef.h>
#include <sys/select.h>
#include <time.h>
#ifndef NO_THREADS
#include <pthread.h>
#endif

/* we assume machine order = network order = le for simplity here */
#define AI2_MEASUREMENT 8
struct __attribute__((__packed__)) measurement_sv {
	uint32_t fcount; /* probably ms since start of report */
	struct __attribute__((__packed__)) {
		uint8_t sv;
		uint16_t snr;
		uint16_t cno;
		uint8_t unknown[23];
	} svdata[];
};

#define AI2_POSITION 6
struct __attribute__((__packed__)) position {
	uint32_t fcount; /* probably ms since start of report */
	uint16_t unknown1;
	int32_t lat;
	int32_t lon;
	int16_t altitude;
	uint8_t unknown2[15];
	struct __attribute__((__packed__)) {
		uint8_t sv;
		uint8_t unknown[5];
	} svdata[];
};

#define AI2_NMEA 0xd3
struct __attribute__((__packed__)) nmea {
	uint32_t fcount;
	char nmea[];
};

#define AI2_POSITION_EXT 0xd5
struct __attribute__((__packed__)) position_ext {
	uint32_t fcount; /* probably ms since start of report */
	uint16_t unknown1;
	int32_t lat;
	int32_t lon;
	uint8_t unknown[47];
	struct __attribute__((__packed__)) {
		uint8_t sv;
		uint8_t unknown[5];
	} svdata[];
};

#define AI2_ERROR 0xf5

static bool nmeaout;
static bool noinit;
static bool noprocess;

__attribute__((__format__ (__printf__, 1, 2)))
static void decode_info_out(const char *format, ...)
{
	va_list ap;

	va_start(ap, format);
	vfprintf(nmeaout ? stderr: stdout, format, ap);
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

static void process_nmea(const uint8_t *data, int len)
{
	const struct nmea *p = (const struct nmea *) data;
	decode_info_out("nmea: fcount: %d:", p->fcount);
	if (len > 4) {
		fwrite(p->nmea, 1, len - 4, stdout);
	}
}

static void process_position_ext(const uint8_t *data, int len)
{
	const struct position_ext *p = (const struct position_ext *) data;
	int i;
	if (len < sizeof(struct position_ext))
	       return;

	double lat = 90 *(double)p->lat / 2147483648.0;
	double lon = 180 *(double)p->lon / 2147483648.0;
	decode_info_out("position: fcount: %d, lat: %f lon: %f",
			p->fcount, lat, lon);
	len -= offsetof(struct position_ext, svdata);
	len /= sizeof(p->svdata[0]);
	decode_info_out(" sv:");
	for(i = 0; i < len; i++)
		decode_info_out(" %d", p->svdata[i].sv);
	decode_info_out("\n");
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
	len -= offsetof(struct position, svdata);
	len /= sizeof(p->svdata[0]);
	decode_info_out(" sv:");
	for(i = 0; i < len; i++)
		decode_info_out(" %d", p->svdata[i].sv);
	decode_info_out("\n");
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

static void process_packet(uint8_t class, uint8_t type, const uint8_t *data, int len)
{
	if (noprocess) {
		int i;
		decode_info_out("0x%02x, 0x%02x, {", class, type);
		for(i = 0; i < len; i++)
			decode_info_out("0x%02x, ", data[i]);

		decode_info_out("}\n");

		decode_info_out("%02x, %02x, ", class, type);
		for(i = 0; i < len; i++)
			decode_info_out("%02x", data[i]);

		decode_info_out("\n");
		return;
	}

	decode_info_out("packet type %x, payload: %d\n", type, len);
	switch(type) {
	case AI2_MEASUREMENT:
		process_measurement(data, len);
		break;
	case AI2_POSITION:
		process_position(data, len);
		break;
	case AI2_NMEA:
		process_nmea(data, len);
		break;
	case AI2_POSITION_EXT:
		process_position_ext(data, len);
		break;
	case AI2_ERROR:
		if (len == 2)
			decode_info_out("got error code %02x %02x\n ", data[0], data[1]);
		else
			decode_info_out("got error with len %d\n", len);
		break;
	default:
		decode_info_out("unknown packet type %x len: %d\n", (int)type, len);
	}
}

static void write_init(int fd, bool nmea)
{
	uint8_t init1[] = {0x10,0x00,0xf5,0x01,0x00,0x01,0x07,0x01,0x10,0x03};
	uint8_t init2[] = {0x10,0x01,0xf1,0x01,0x00,0x05,0x08,0x01,0x10,0x03};
	uint8_t init3[] = {0x10,0x01,0xf0,0x00,0x00,0x01,0x01,0x10,0x03};
	uint8_t init4[] = {0x10,0x01,0x02,0x01,0x00,0x02,0x16,0x00,0x10,0x03}; // C_RECEIVER_IDLE
	uint8_t init5[] = {0x10,0x01,0xed,0x01,0x00,0x00,0xff,0x00,0x10,0x03};
	uint8_t init6[] = {0x10,0x01,0x06,0x0d,0x00,0x01,0x0e,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x34,0x00,0x10,0x03};
	uint8_t init7[] = {0x10,0x01,0x02,0x01,0x00,0x03,0x17,0x00,0x10,0x03}; // C_RECEIVER_ON

	uint8_t init_nmea[] = {
		0x10, 0x01, 0x08, 0x18, 0x00, 0x00, 0x01, 0x3c, 0x01, 0x00, 0x01, 0x04,
		0x83, 0x03, 0x70, 0x17, 0xa0, 0x0f, 0x07, 0x1e, 0x07, 0x1e, 0x01, 0x00,
		0x00, 0x00, 0x00, 0x01, 0x00,  // end of msg 1
		0x06, 0x0d, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,  // msg 2 similar init 6
		0x20, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57, 0x02, 0x00, 0x00, 0x01,  // msg 3
		0xe5, 0x04, 0x00, 0x3f, 0x00, 0x00, 0x00,  // msg 4
		0x02, 0x01, 0x00, 0x03, 0x42, 0x05, 0x10, 0x03 // msg 5 C_RECEIVER_ON
	};

	uint8_t init_nmea2[] = {
  0x10, 0x00, 0x22, 0x01, 0x00, 0x01, 0x34, 0x00, 0x10, 0x03, // starts nmea
  0x10, 0x00, 0x09, 0x00, 0x00, 0x19, 0x00, 0x10, 0x03,
  0x10, 0x00, 0x06, 0x0d, 0x00, 0x01, 0x20, 0x00, 0x00, 0xc0, 0xff, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x05, 0x02, 0x10, 0x03,
  0x10, 0x00, 0xfb, 0x16, 0x00, 0x15, 0x00, 0x28, 0x00, 0x50, 0x00, 0x50, 0x00, 0x96, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x94, 0x02, 0x10, 0x03
	};

	uint8_t init_nmea_rep[] = {0x10, 0x00, 0x22, 0x01, 0x00, 0x01, 0x34, 0x00, 0x10, 0x03};

	write(fd, init1, sizeof(init1));
	usleep(200000);
	write(fd, init2, sizeof(init2));
	usleep(200000);
	if (nmea) {
#if 1
		write(fd, init_nmea, sizeof(init_nmea));
		usleep(200000);
#endif
		write(fd, init_nmea_rep, sizeof(init_nmea_rep));
		usleep(200000);
#if 0
        while(1) {
	  write(fd, init_nmea_rep, sizeof(init_nmea_rep));
	  usleep(2000000);
	}
#endif
        } else {
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
}

static void process_ai2_frame(uint8_t *buf, size_t len)
{
	uint16_t sum;
	uint16_t chk;
	uint8_t class, type;
	size_t i;
	if (len < 4)
		return;

	chk = buf[len - 1];
	chk <<= 8;

	chk |= buf[len - 2];
	len -= 2;

	for(i = 0, sum = 0; i < len; i++)
		sum += buf[i];

	if (chk != sum) {
		decode_err_out("checksum mismatch %04x != %04x\n", (int)chk, (int)sum);
		return;
	}

	class = buf[1];

	if (class == 2) {
		decode_info_out("decoded ack\n");
		return;
	}
	buf += 2;
	len -= 2;

	while(len >= 3) {
		uint8_t type;
		uint16_t sublen;
		type = buf[0];
		sublen = buf[2];
		sublen <<= 8;
		sublen |= buf[1];
		buf += 3;
		len -= 3;
		if (len < sublen) {
			decode_err_out("packet cut off\n");
			break;
		}
		process_packet(class, type, buf, sublen);
		buf += sublen;
		len -= sublen;
	}
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
	bool escaping;
	uint16_t sum;
	size_t pktlen;
	f = fdopen(fd, "r+");
	bufpos = 0;
	pktlen = 0;
	while((c = fgetc(f)) != EOF) {
		totalbufpos++;
		if (bufpos == 0) {
			escaping = false;
			if (c != 0x10) {
				decode_err_out("d");
				continue;
			}
			decode_err_out("\n");
		}

		if (bufpos == 1) {
			if (c == 3) {
				decode_err_out("%04x unexpected end of packet\n", totalbufpos);
				bufpos = 0;
				continue;
			}
		}

		if ((!escaping) && (c == 0x10) && (bufpos != 0)) {
			escaping = true;
			continue;
		}

		if (escaping && (c == 3)) {
			process_ai2_frame(gpsbuf, bufpos);
			bufpos = 0;
			continue;
		}
		escaping = false;
		if (bufpos == sizeof(gpsbuf)) {
			bufpos = 0;
			decode_err_out("overlong packet, throwing away\n");
		}
		gpsbuf[bufpos] = c;
		bufpos++;
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

	if (argc > 2) {
		if (!strcmp(argv[2], "nmea"))
			nmeaout = true;

		if (!strcmp(argv[2], "noinit"))
			noinit = true;

		if (!strcmp(argv[2], "noprocess")) {
			noinit = true;
			noprocess = true;
		}
	}

        fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Cannot open %s\n", argv[1]);
		return 1;
	}

#ifndef NO_THREADS
	pthread_t thread;
	pthread_create(&thread, NULL, read_loop, &fd);
#endif
	if (!noinit)
		write_init(fd, nmeaout);
#ifndef NO_THREADS
	pthread_join(thread, NULL);
	return 0;
#else
	read_loop(&fd);
	return 0;
#endif
}
