/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Dezhi Wu
 *
 * Test tool for the IPTS HID device: read or write a feature report.
 *
 *   iptsctl get ID LEN       read feature report ID (LEN bytes with ID)
 *   iptsctl set ID BYTE...   write feature report ID with the bytes
 */

#include <sys/types.h>
#include <sys/ioctl.h>

#include <dev/hid/hidraw.h>

#include <err.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define	IPTS_VENDOR	0x045e
#define	IPTS_PRODUCT	0x099f

/* Find and open the hidraw device of IPTS. */
static int
ipts_open(void)
{
	struct hidraw_devinfo info;
	char path[32];
	int fd, i;

	for (i = 0; i < 16; i++) {
		snprintf(path, sizeof(path), "/dev/hidraw%d", i);
		if ((fd = open(path, O_RDWR)) < 0)
			continue;
		if (ioctl(fd, HIDIOCGRAWINFO, &info) == 0 &&
		    (uint16_t)info.vendor == IPTS_VENDOR &&
		    (uint16_t)info.product == IPTS_PRODUCT) {
			fprintf(stderr, "using %s\n", path);
			return (fd);
		}
		close(fd);
	}
	errx(1, "no IPTS hidraw device");
}

int
main(int argc, char **argv)
{
	unsigned char buf[4096];
	int fd, i, len, n;

	if (argc < 4)
		errx(1, "usage: iptsctl get ID LEN | set ID BYTE...");
	fd = ipts_open();
	memset(buf, 0, sizeof(buf));
	buf[0] = strtoul(argv[2], NULL, 0);

	if (strcmp(argv[1], "get") == 0) {
		len = strtoul(argv[3], NULL, 0);
		if (len < 1 || len > (int)sizeof(buf))
			errx(1, "bad length");
		n = ioctl(fd, HIDIOCGFEATURE(len), buf);
		if (n < 0)
			err(1, "HIDIOCGFEATURE");
		printf("report %#x, %d bytes:", buf[0], n);
		for (i = 0; i < n; i++)
			printf("%s%02x", i % 16 == 0 ? "\n  " : " ", buf[i]);
		printf("\n");
	} else if (strcmp(argv[1], "set") == 0) {
		len = argc - 2;
		if (len > (int)sizeof(buf))
			errx(1, "too many bytes");
		for (i = 3; i < argc; i++)
			buf[i - 2] = strtoul(argv[i], NULL, 0);
		if (ioctl(fd, HIDIOCSFEATURE(len), buf) < 0)
			err(1, "HIDIOCSFEATURE");
		printf("report %#x set, %d bytes\n", buf[0], len);
	} else
		errx(1, "unknown command %s", argv[1]);
	close(fd);
	return (0);
}
