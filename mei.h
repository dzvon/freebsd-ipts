/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Dezhi Wu
 *
 * Interface of the Intel MEI driver for ME client drivers.
 */

#ifndef _MEI_H_
#define _MEI_H_

/* Properties of an ME client. */
struct mei_client_props {
	uint8_t		addr;		/* ME address */
	uint8_t		uuid[16];	/* protocol name, little endian */
	uint8_t		version;
	uint8_t		max_conn;
	uint8_t		fixed;
	uint8_t		single_recv;
	uint32_t	max_msg;
};

/*
 * The MEI driver adds one child device for each ME client.  The
 * functions below take that child device.  All of them can sleep.
 * A timeout is in ticks.
 */
const struct mei_client_props *mei_cl_props(device_t dev);
int	mei_cl_connect(device_t dev);
int	mei_cl_disconnect(device_t dev);
int	mei_cl_send(device_t dev, const void *buf, size_t len, int timo);
int	mei_cl_recv(device_t dev, void *buf, size_t maxlen, size_t *len,
	    int timo);

/* Compare a client UUID with a UUID in text form. */
int	mei_uuid_match(const uint8_t *uuid, const char *str);

#endif /* _MEI_H_ */
