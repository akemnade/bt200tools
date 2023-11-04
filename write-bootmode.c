// SPDX-License-Identifier: MIT
/*
 * write bootmode to be used by ancient u-boots like the one
 * in the bt200 to decide whether to boot or not
 */
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>

int main(int argc, char **argv)
{
	int fd;
	
	if ((argc < 2) || (!strcmp("--help", argv[1]))) {
		fprintf(stderr, "%s bootmode\n"
			"interesting values for bootmode are normal,\n"
			"normal_boot and are read out by factory u-boot\n",
			argv[0]);
		return 1;
	}

	fd = open("/dev/mem", O_RDWR);
        if (fd < 0)  {
		perror("open: ");
		return 1;
	}

        uint8_t *data = mmap(NULL, 0x1000, PROT_READ |  PROT_WRITE, MAP_SHARED, fd, 0x4a326000);
	if (data == NULL) {
		perror("mmap: ");
		return 1;
	}

	strcpy(data + 0xA0C, argv[1]);
        return 0;
}
