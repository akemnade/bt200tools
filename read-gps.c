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
#include <ctype.h>
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

#define AI2_ASYNC_EVENT 0x80
#define AI2_ASYNC_EVENT_ENG_IDLE 0x07
#define AI2_ASYNC_EVENT_ENG_OFF 0x01

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

static void process_async_event(const uint8_t *data, int len)
{
	switch(data[0]) {
		case AI2_ASYNC_EVENT_ENG_IDLE:
			decode_info_out("Event: machine idle\n");
			break;
		case AI2_ASYNC_EVENT_ENG_OFF:
			decode_info_out("Event: machine off\n");
			break;
		default:
			decode_info_out("Event: unknown (%02x)\n", data[0]);

	}
}

static void dump_packet(uint8_t class, uint8_t type, const uint8_t *data, int len)
{
	int i;
	decode_info_out("0x%02x, 0x%02x, {", class, type);
	for(i = 0; i < len; i++)
		decode_info_out("0x%02x, ", data[i]);

	decode_info_out("}\n");

	decode_info_out("%02x, %02x, ", class, type);
	for(i = 0; i < len; i++)
		decode_info_out("%02x", data[i]);

	decode_info_out("\n");
}

static void process_packet(uint8_t class, uint8_t type, const uint8_t *data, int len)
{
	int i;
	if (noprocess) {
		dump_packet(class, type, data, len);
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
	case AI2_ASYNC_EVENT:
		process_async_event(data, len);
		break;
	case AI2_ERROR:
		if (len == 2) {
			uint16_t err = data[1];
			err <<= 8;
			err |= data[0];
			switch(err) {
				case 0x02ff:
					decode_info_out("error invalid checksum\n ", err);
					break;
				default:
					decode_info_out("got error code %04x\n ", err);
			}
		} else
			decode_info_out("got error with len %d\n", len);
		break;
	default:
		decode_info_out("unknown packet type %x len: %d ", (int)type, len);
		for(i = 0; i < len; i++) {
			decode_info_out("%02x", data[i]);
		}
		decode_info_out("\n");
	}
}

static int append_packet(uint8_t *pkt, int pktpos, uint8_t data)
{
	pkt[pktpos] = data;
	pktpos++;
	if (data == 0x10) {
		pkt[pktpos] = data;
		pktpos++;
	}
	return pktpos;
}

static int write_packet(int fd, uint8_t class, uint8_t cmd, uint8_t *data, uint16_t len)
{
	uint8_t *pkt = calloc(1, 4 + len * 2  + 2);
	int i;
	int ret;
	uint16_t sum;
	int pktpos = 2;
	pkt[0] = 0x10;
	pkt[1] = class;

	sum = 0x10 + class + cmd;
	sum += len & 0xff;
	sum += len >> 8;

	pktpos = append_packet(pkt, pktpos, cmd);
	pktpos = append_packet(pkt, pktpos, len & 0xff);
	pktpos = append_packet(pkt, pktpos, len >> 8);

	for (i = 0; i < len; i++) {
		pktpos = append_packet(pkt, pktpos, data[i]);
		sum += data[i];
	}

	pkt[pktpos] = sum & 0xff;
	pktpos++;

	pkt[pktpos] = sum >> 8;
	pktpos++;

	pkt[pktpos] = 0x10;
	pktpos++;
	pkt[pktpos] = 0x03;
	pktpos++;

	ret = write(fd, pkt, pktpos);

	free(pkt);
	return ret;
}

#define RECEIVER_STATE_OFF 1
#define RECEIVER_STATE_IDLE 2
#define RECEIVER_STATE_ON 3
static int set_receiver_state(int fd, uint8_t state)
{
	return write_packet(fd, 1, 2, &state, 1);
}

#define NMEA_MASK_GGA (1 << 0)
#define NMEA_MASK_GLL (1 << 1)
#define NMEA_MASK_GSA (1 << 2)
#define NMEA_MASK_GSV (1 << 3)
#define NMEA_MASK_RMC (1 << 4)
#define NMEA_MASK_VTG (1 << 5)

#define NMEA_MASK_ALL (NMEA_MASK_GGA | NMEA_MASK_GLL | NMEA_MASK_GSA | NMEA_MASK_GSV | NMEA_MASK_RMC | NMEA_MASK_VTG)

static int enable_nmea_reports(int fd, uint8_t mask)
{
	uint8_t buf[4] = {0};
	buf[0] = mask;
	return write_packet(fd, 1, 0xe5, buf, sizeof(buf));
}

#define WRITE_PKT(fd, class, type, data) do { uint8_t d[] = data; write_packet(fd, (class), (type), d, sizeof(d)); } while(0)

static void write_init(int fd, bool nmea)
{
#if 0
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
#endif

	WRITE_PKT(fd, 0x00,0xf5, {0x01});
	usleep(200000);
	WRITE_PKT(fd, 0x01,0xf1, {0x05});
	usleep(200000);

	set_receiver_state(fd, RECEIVER_STATE_IDLE);
	usleep(200000);
	if (nmea) {
		enable_nmea_reports(fd, NMEA_MASK_ALL);
		usleep(200000);
		set_receiver_state(fd, RECEIVER_STATE_ON);
#if 0
        while(1) {
	  write(fd, init_nmea_rep, sizeof(init_nmea_rep));
	  usleep(2000000);
	}
#endif
        } else {
		WRITE_PKT(fd, 0x01, 0xf0, {});
		usleep(200000);
		set_receiver_state(fd, RECEIVER_STATE_IDLE);
		usleep(200000);
		WRITE_PKT(fd, 0x01, 0xed, {0x00});
		usleep(200000);
		uint8_t pk[] = {0x01,0x0e,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
		write_packet(fd, 0x01, 0x06, pk, sizeof(pk));
		usleep(200000);
		set_receiver_state(fd, RECEIVER_STATE_ON);
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
	f = fdopen(fd, "r");
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


static int hexbuf_to_str(const char *src, uint8_t *dest, int len)
{
	char buf[5];
	int i;
	int c;
	bool first = true;
	int destpos = 0;
	buf[4] = 0;
	buf[0] = '0';
	buf[1] = 'x';
	for(i=0; i < len; i++) {
		if (!isxdigit(src[i])) {
			first = true;
			continue;
		}
		if (first) {
			buf[2] = src[i];
			first = false;
		} else {
			buf[3] = src[i];
			first = true;
			dest[destpos] = strtoul(buf, NULL, 0);
			destpos++;
		}
	}
	return destpos;
}

void hex_from_stdin_to(int fd)
{
	char buf[1024];
	while(fgets(buf, sizeof(buf), stdin)) {
		int l = hexbuf_to_str(buf, buf, strlen(buf));
		write(fd, buf, l);
	}
}

void cmd_from_stdin_to(int fd)
{
	char buf[1024];
	while(fgets(buf, sizeof(buf), stdin)) {
		unsigned int class, cmd;
		char *data;
		if (sscanf(buf, "%x %x %ms", &class, &cmd, &data) == 3) {
			int l = hexbuf_to_str(data, data, strlen(buf));
			write_packet(fd, class, cmd, data, l);
			free(data);
		}
	}
}


int main(int argc, char **argv)
{
	int fd;
	struct timeval tv;
	fd_set fds;
	bool send_off = false;
	bool send_idle = false;
	int pipefds[2] = {-1};
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

		if (!strcmp(argv[2], "off")) {
			noinit = true;
			send_idle = true;
			send_off = true;
		}

		if (!strcmp(argv[2], "idle")) {
			noinit = true;
			send_idle = true;
		}
	}

	if (!strcmp(argv[1], "-")) {
		pipe(pipefds);
		noinit = true;
		fd = pipefds[0];
	} else {
		fd = open(argv[1], O_RDWR);
	}
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

	if (send_idle) {
		WRITE_PKT(fd, 1, 2, {2});
		/* we need to wait for idle state */
		usleep(500000);
	}

	/* this keeps satellite data, rmmod does not! */
	if (send_off) {
		WRITE_PKT(fd, 1, 2, {1});
		usleep(500000);
		return 0;
	}

	if (pipefds[1] > 0) {
		hex_from_stdin_to(pipefds[1]);
		close(pipefds[1]);
	}
#ifndef NO_THREADS
	else if (noinit) {
		cmd_from_stdin_to(fd);
		return 0;
	}

	pthread_join(thread, NULL);
	return 0;
#else
	read_loop(&fd);
	return 0;
#endif
}
