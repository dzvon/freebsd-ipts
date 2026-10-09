/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Dezhi Wu
 *
 * Driver for the Intel Management Engine Interface (MEI, also HECI).
 *
 * The host and the ME exchange messages through two circular buffers
 * in MMIO space: one that the host writes and one that the ME writes.
 * Each message has a 32-bit header with the ME client address, the
 * host client address and the length.  Messages between address 0 and
 * address 0 are Host Bus Messages (HBM).  The host uses HBM to start
 * the interface, to list the ME clients and to connect to them.
 *
 * Flow control: a side can send one message to a client only after it
 * gets a flow control credit for that client from the other side.
 *
 * The driver adds a child device for each ME client.  Client drivers
 * use the functions in mei.h to exchange messages with their client.
 *
 * It attaches only to the "iTouch" MEI device of Ice Lake (8086:34e4),
 * which Intel Precise Touch & Stylus (IPTS) uses on the Surface Pro 7.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sbuf.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "mei.h"

/* MMIO registers. */
#define	MEI_H_CB_WW	0x00	/* host circular buffer write window */
#define	MEI_H_CSR	0x04	/* host control and status */
#define	MEI_ME_CB_RW	0x08	/* ME circular buffer read window */
#define	MEI_ME_CSR	0x0c	/* ME control and status */
#define	MEI_H_D0I3C	0x800	/* D0i3 control */

/* Bits of H_CSR and ME_CSR. */
#define	CSR_IE		0x00000001	/* interrupt enable */
#define	CSR_IS		0x00000002	/* interrupt status */
#define	CSR_IG		0x00000004	/* interrupt generate */
#define	CSR_RDY		0x00000008	/* ready */
#define	CSR_RST		0x00000010	/* reset */
#define	H_CSR_D0I3_IE	0x00000020
#define	H_CSR_D0I3_IS	0x00000040
#define	H_CSR_IE_MASK	(CSR_IE | H_CSR_D0I3_IE)
#define	H_CSR_IS_MASK	(CSR_IS | H_CSR_D0I3_IS)
#define	CSR_CBRP(x)	(((x) >> 8) & 0xff)	/* read pointer */
#define	CSR_CBWP(x)	(((x) >> 16) & 0xff)	/* write pointer */
#define	CSR_CBD(x)	(((x) >> 24) & 0xff)	/* depth in dwords */

/* Bits of H_D0I3C. */
#define	D0I3C_CIP	0x00000001	/* command in progress */
#define	D0I3C_IR	0x00000002	/* interrupt required */
#define	D0I3C_I3	0x00000004	/* in D0i3 */

/* PCI configuration space: firmware status registers. */
#define	MEI_PCI_HFS_1	0x40
#define	MEI_PCI_HFS_2	0x48
#define	MEI_PCI_HFS_3	0x60
#define	MEI_PCI_HFS_4	0x64
#define	MEI_PCI_HFS_5	0x68
#define	MEI_PCI_HFS_6	0x6c
#define	MEI_HFS_1_D0I3	0x80000000	/* D0i3 is supported */

/* Message header. */
#define	MEI_HDR(me, host, len)						\
	((uint32_t)(me) | ((uint32_t)(host) << 8) |			\
	(((uint32_t)(len) & 0x1ff) << 16) | 0x80000000u)
#define	MEI_HDR_ME(h)		((h) & 0xff)
#define	MEI_HDR_HOST(h)		(((h) >> 8) & 0xff)
#define	MEI_HDR_LEN(h)		(((h) >> 16) & 0x1ff)
#define	MEI_HDR_COMPLETE(h)	(((h) >> 31) & 1)
#define	MEI_MAX_MSG		0x1ff

/* Host Bus Message commands. */
#define	HBM_START_REQ		0x01
#define	HBM_ENUM_REQ		0x04
#define	HBM_PROPS_REQ		0x05
#define	HBM_CONNECT_REQ		0x06
#define	HBM_DISCONNECT_REQ	0x07
#define	HBM_FLOW_CONTROL	0x08
#define	HBM_RES			0x80
#define	HBM_START_RES_LEN	4
#define	HBM_ENUM_RES_LEN	36
#define	HBM_PROPS_RES_LEN	28
#define	HBM_CONNECT_RES_LEN	4
#define	HBM_FLOW_CONTROL_LEN	8

#define	HBM_VERSION_MAJOR	2
#define	HBM_VERSION_MINOR	0

#define	MEI_READY_TIMEOUT	(2 * hz)
#define	MEI_HBM_TIMEOUT		(5 * hz)
#define	MEI_POLL		MAX(1, hz / 100)

/* Number of received messages that a client can keep. */
#define	MEI_RXQ_LEN		8

struct mei_rx_msg {
	size_t		len;
	uint8_t		data[MEI_MAX_MSG + 1];
};

struct mei_client {
	struct mei_client_props	props;
	uint8_t			host_addr;
	device_t		child;
	int			connected;
	int			tx_credits;	/* we can send */
	int			rx_credit;	/* the ME can send */
	struct mei_rx_msg	rxq[MEI_RXQ_LEN];
	u_int			rxq_head;
	u_int			rxq_count;
	u_long			st_rx;
	u_long			st_rx_drop;
	u_long			st_tx;
};

struct mei_softc {
	device_t		dev;
	struct resource		*mem;
	int			mem_rid;
	struct resource		*irq;
	int			irq_rid;
	void			*intr_cookie;
	int			msi;

	struct mtx		mtx;
	struct sx		hbm_lock;	/* one HBM request at a time */
	int			d0i3_supported;
	int			ready;
	uint8_t			hbm_major;
	uint8_t			hbm_minor;

	/* HBM response that a request waits for. */
	uint8_t			hbm_wait;	/* command, 0 if none */
	int			hbm_done;
	uint8_t			hbm_rsp[MEI_MAX_MSG + 1];
	size_t			hbm_rsp_len;

	struct mei_client	*clients;
	int			nclients;

	uint32_t		hfs[6];
	uint32_t		rbuf[(MEI_MAX_MSG + 3) / 4];

	u_long			st_intr;
	u_long			st_rx_msgs;
	u_long			st_rx_hbm_other;
	u_long			st_rx_client_unknown;
	u_long			st_rx_errors;
	u_long			st_me_resets;
};

static MALLOC_DEFINE(M_MEI, "mei", "Intel MEI driver");

static int	mei_detach(device_t dev);

#define	RD4(sc, reg)		bus_read_4((sc)->mem, (reg))
#define	WR4(sc, reg, val)	bus_write_4((sc)->mem, (reg), (val))

/* Write H_CSR without a change to the interrupt status bits. */
static void
mei_hcsr_set(struct mei_softc *sc, uint32_t hcsr)
{
	WR4(sc, MEI_H_CSR, hcsr & ~H_CSR_IS_MASK);
}

static int
mei_wait_reg(struct mei_softc *sc, bus_size_t reg, uint32_t mask,
    uint32_t val, int timo)
{
	int t;

	for (t = 0; t < timo; t += MEI_POLL) {
		if ((RD4(sc, reg) & mask) == val)
			return (0);
		msleep(sc, &sc->mtx, 0, "meireg", MEI_POLL);
	}
	return ((RD4(sc, reg) & mask) == val ? 0 : ETIMEDOUT);
}

/*
 * Send.
 */
static int
mei_write(struct mei_softc *sc, uint8_t me_addr, uint8_t host_addr,
    const void *data, size_t len)
{
	const uint8_t *p = data;
	uint32_t hcsr, dw;
	u_int depth, filled, n;
	size_t chunk, i;

	mtx_assert(&sc->mtx, MA_OWNED);
	if (len > MEI_MAX_MSG)
		return (EINVAL);
	if ((RD4(sc, MEI_ME_CSR) & CSR_RDY) == 0)
		return (EIO);

	hcsr = RD4(sc, MEI_H_CSR);
	depth = CSR_CBD(hcsr);
	filled = (uint8_t)(CSR_CBWP(hcsr) - CSR_CBRP(hcsr));
	if (filled > depth)
		return (EIO);
	n = 1 + howmany(len, 4);
	if (n > depth - filled)
		return (EAGAIN);

	WR4(sc, MEI_H_CB_WW, MEI_HDR(me_addr, host_addr, len));
	for (i = 0; i < len; i += 4) {
		chunk = MIN(4, len - i);
		dw = 0;
		memcpy(&dw, p + i, chunk);
		WR4(sc, MEI_H_CB_WW, le32toh(dw));
	}
	mei_hcsr_set(sc, RD4(sc, MEI_H_CSR) | CSR_IG);
	return (0);
}

/* Give the ME a credit to send one message to the client.  Lock held. */
static void
mei_send_flow_control(struct mei_softc *sc, struct mei_client *cl)
{
	uint8_t msg[HBM_FLOW_CONTROL_LEN];

	if (!cl->connected || cl->props.fixed || cl->rx_credit ||
	    cl->rxq_count >= MEI_RXQ_LEN)
		return;
	memset(msg, 0, sizeof(msg));
	msg[0] = HBM_FLOW_CONTROL;
	msg[1] = cl->props.addr;
	msg[2] = cl->host_addr;
	if (mei_write(sc, 0, 0, msg, sizeof(msg)) == 0)
		cl->rx_credit = 1;
}

/*
 * Receive.
 */
static struct mei_client *
mei_find_client(struct mei_softc *sc, uint8_t me_addr)
{
	int i;

	for (i = 0; i < sc->nclients; i++)
		if (sc->clients[i].props.addr == me_addr)
			return (&sc->clients[i]);
	return (NULL);
}

static void
mei_rx_hbm(struct mei_softc *sc, const uint8_t *msg, size_t len)
{
	struct mei_client *cl;
	uint8_t rsp[4];

	if (len < 1)
		return;

	switch (msg[0]) {
	case HBM_FLOW_CONTROL:
		/* The credit can come before we see the connect response. */
		if (len >= 3 && (cl = mei_find_client(sc, msg[1])) != NULL) {
			cl->tx_credits++;
			wakeup(&cl->tx_credits);
		}
		return;
	case HBM_DISCONNECT_REQ:
		/* The ME closes the connection. */
		if (len >= 3 && (cl = mei_find_client(sc, msg[1])) != NULL) {
			device_printf(sc->dev, "client %u: ME disconnects\n",
			    cl->props.addr);
			cl->connected = 0;
			cl->tx_credits = 0;
			cl->rx_credit = 0;
			wakeup(&cl->tx_credits);
			wakeup(&cl->rxq_count);
			rsp[0] = HBM_DISCONNECT_REQ | HBM_RES;
			rsp[1] = msg[1];
			rsp[2] = msg[2];
			rsp[3] = 0;
			(void)mei_write(sc, 0, 0, rsp, sizeof(rsp));
		}
		return;
	}

	if (sc->hbm_wait != 0 && msg[0] == sc->hbm_wait && !sc->hbm_done) {
		memcpy(sc->hbm_rsp, msg, len);
		sc->hbm_rsp_len = len;
		sc->hbm_done = 1;
		wakeup(&sc->hbm_done);
		return;
	}
	sc->st_rx_hbm_other++;
	if (bootverbose)
		device_printf(sc->dev, "HBM message %#x, %zu bytes\n",
		    msg[0], len);
}

static void
mei_rx_client(struct mei_softc *sc, uint32_t hdr, const uint8_t *msg,
    size_t len)
{
	struct mei_client *cl;
	struct mei_rx_msg *m;

	cl = mei_find_client(sc, MEI_HDR_ME(hdr));
	if (cl == NULL || !cl->connected ||
	    MEI_HDR_HOST(hdr) != cl->host_addr) {
		sc->st_rx_client_unknown++;
		return;
	}
	cl->rx_credit = 0;
	if (cl->rxq_count >= MEI_RXQ_LEN || !MEI_HDR_COMPLETE(hdr)) {
		/* The message is too long or does not fit. */
		cl->st_rx_drop++;
	} else {
		m = &cl->rxq[(cl->rxq_head + cl->rxq_count) % MEI_RXQ_LEN];
		memcpy(m->data, msg, len);
		m->len = len;
		cl->rxq_count++;
		cl->st_rx++;
		wakeup(&cl->rxq_count);
	}
	mei_send_flow_control(sc, cl);
}

/* Read all messages from the ME circular buffer.  Lock held. */
static void
mei_process(struct mei_softc *sc)
{
	uint32_t hcsr, mecsr, hdr;
	u_int depth, filled, n, i;
	size_t len;

	mtx_assert(&sc->mtx, MA_OWNED);

	hcsr = RD4(sc, MEI_H_CSR);
	if (hcsr & H_CSR_IS_MASK)
		WR4(sc, MEI_H_CSR, hcsr);	/* clear the status bits */

	mecsr = RD4(sc, MEI_ME_CSR);
	if (mecsr & CSR_RST) {
		sc->st_me_resets++;
		mei_hcsr_set(sc, RD4(sc, MEI_H_CSR) | CSR_IG);
		return;
	}
	/* Do not read the buffer before the reset is complete. */
	if ((hcsr & CSR_RDY) == 0 || (mecsr & CSR_RDY) == 0)
		return;

	for (;;) {
		mecsr = RD4(sc, MEI_ME_CSR);
		depth = CSR_CBD(mecsr);
		filled = (uint8_t)(CSR_CBWP(mecsr) - CSR_CBRP(mecsr));
		if (filled == 0)
			break;
		if (filled > depth) {
			sc->st_rx_errors++;
			break;
		}
		hdr = RD4(sc, MEI_ME_CB_RW);
		len = MEI_HDR_LEN(hdr);
		n = howmany(len, 4);
		if (n > filled - 1) {
			/* The ME writes a full message at a time. */
			sc->st_rx_errors++;
			break;
		}
		for (i = 0; i < n; i++)
			sc->rbuf[i] = htole32(RD4(sc, MEI_ME_CB_RW));
		sc->st_rx_msgs++;

		if (MEI_HDR_ME(hdr) == 0 && MEI_HDR_HOST(hdr) == 0)
			mei_rx_hbm(sc, (uint8_t *)sc->rbuf, len);
		else
			mei_rx_client(sc, hdr, (uint8_t *)sc->rbuf, len);

		/* Tell the ME that we read the slots. */
		mei_hcsr_set(sc, RD4(sc, MEI_H_CSR) | CSR_IG);
	}
}

static void
mei_intr(void *arg)
{
	struct mei_softc *sc = arg;

	mtx_lock(&sc->mtx);
	sc->st_intr++;
	mei_process(sc);
	mtx_unlock(&sc->mtx);
}

/*
 * Send an HBM request and wait for the response.  The response
 * command is the request command with bit 7 set.  Lock held.
 */
static int
mei_hbm_request(struct mei_softc *sc, const void *req, size_t len,
    size_t rsp_min)
{
	int error, t;

	mtx_assert(&sc->mtx, MA_OWNED);
	sc->hbm_wait = ((const uint8_t *)req)[0] | HBM_RES;
	sc->hbm_done = 0;
	sc->hbm_rsp_len = 0;

	for (t = 0; t < 10; t++) {
		error = mei_write(sc, 0, 0, req, len);
		if (error != EAGAIN)
			break;
		msleep(sc, &sc->mtx, 0, "meiwr", MEI_POLL);
	}
	if (error != 0)
		goto out;

	/* Poll as well, in case the interrupt does not come. */
	for (t = 0; t < MEI_HBM_TIMEOUT && !sc->hbm_done; t += MEI_POLL) {
		mei_process(sc);
		if (sc->hbm_done)
			break;
		msleep(&sc->hbm_done, &sc->mtx, 0, "meihbm", MEI_POLL);
	}
	if (!sc->hbm_done)
		error = ETIMEDOUT;
	else if (sc->hbm_rsp_len < rsp_min)
		error = EIO;
out:
	sc->hbm_wait = 0;
	if (error != 0)
		device_printf(sc->dev, "HBM request %#x failed: %d\n",
		    ((const uint8_t *)req)[0], error);
	return (error);
}

/*
 * Start.
 */
static int
mei_d0i3_exit(struct mei_softc *sc)
{
	uint32_t reg;

	if (!sc->d0i3_supported)
		return (0);
	reg = RD4(sc, MEI_H_D0I3C);
	if ((reg & D0I3C_I3) == 0)
		return (0);
	device_printf(sc->dev, "leaving D0i3 (D0I3C %#x)\n", reg);
	WR4(sc, MEI_H_D0I3C, reg & ~(D0I3C_I3 | D0I3C_IR));
	if (mei_wait_reg(sc, MEI_H_D0I3C, D0I3C_CIP | D0I3C_I3, 0,
	    MEI_READY_TIMEOUT) != 0) {
		device_printf(sc->dev, "cannot leave D0i3 (D0I3C %#x)\n",
		    RD4(sc, MEI_H_D0I3C));
		return (ETIMEDOUT);
	}
	return (0);
}

static int
mei_hw_start(struct mei_softc *sc)
{
	uint32_t hcsr;
	int error;

	mtx_assert(&sc->mtx, MA_OWNED);

	if ((error = mei_d0i3_exit(sc)) != 0)
		return (error);

	if (bootverbose)
		device_printf(sc->dev, "before reset: H_CSR %#x ME_CSR %#x\n",
		    RD4(sc, MEI_H_CSR), RD4(sc, MEI_ME_CSR));

	/* A reset that did not complete can leave H_RST set. */
	hcsr = RD4(sc, MEI_H_CSR);
	if (hcsr & CSR_RST) {
		mei_hcsr_set(sc, hcsr & ~CSR_RST);
		hcsr = RD4(sc, MEI_H_CSR);
	}

	/* Reset the interface. */
	hcsr |= CSR_RST | CSR_IG | H_CSR_IS_MASK | H_CSR_IE_MASK;
	hcsr &= ~CSR_RDY;
	WR4(sc, MEI_H_CSR, hcsr);
	(void)RD4(sc, MEI_H_CSR);

	/* The ME clears its ready bit, resets, and then sets it again. */
	if (mei_wait_reg(sc, MEI_ME_CSR, CSR_RDY, 0, hz / 2) != 0)
		device_printf(sc->dev, "ME stays ready during reset\n");
	if (mei_wait_reg(sc, MEI_ME_CSR, CSR_RDY, CSR_RDY,
	    MEI_READY_TIMEOUT) != 0) {
		device_printf(sc->dev, "ME is not ready: ME_CSR %#x\n",
		    RD4(sc, MEI_ME_CSR));
		return (ETIMEDOUT);
	}

	/* Release the reset and tell the ME that the host is ready. */
	hcsr = RD4(sc, MEI_H_CSR);
	hcsr &= ~CSR_RST;
	hcsr |= CSR_IG;
	mei_hcsr_set(sc, hcsr);
	hcsr = RD4(sc, MEI_H_CSR);
	hcsr |= H_CSR_IE_MASK | CSR_IG | CSR_RDY;
	mei_hcsr_set(sc, hcsr);

	if (bootverbose)
		device_printf(sc->dev, "after reset: H_CSR %#x ME_CSR %#x\n",
		    RD4(sc, MEI_H_CSR), RD4(sc, MEI_ME_CSR));
	return (0);
}

static int
mei_hbm_start(struct mei_softc *sc)
{
	uint8_t req[4];
	int error, try;

	sc->hbm_major = HBM_VERSION_MAJOR;
	sc->hbm_minor = HBM_VERSION_MINOR;
	for (try = 0; try < 2; try++) {
		req[0] = HBM_START_REQ;
		req[1] = 0;
		req[2] = sc->hbm_minor;
		req[3] = sc->hbm_major;
		error = mei_hbm_request(sc, req, sizeof(req),
		    HBM_START_RES_LEN);
		if (error != 0)
			return (error);
		if (sc->hbm_rsp[1]) {
			device_printf(sc->dev, "HBM version %u.%u (ME %u.%u)\n",
			    sc->hbm_major, sc->hbm_minor, sc->hbm_rsp[3],
			    sc->hbm_rsp[2]);
			return (0);
		}
		/* Try again with the version of the ME. */
		sc->hbm_major = sc->hbm_rsp[3];
		sc->hbm_minor = sc->hbm_rsp[2];
	}
	device_printf(sc->dev, "no common HBM version\n");
	return (ENXIO);
}

static void
mei_uuid_str(const uint8_t *u, char *buf, size_t len)
{
	snprintf(buf, len, "%08x-%04x-%04x-%02x%02x-"
	    "%02x%02x%02x%02x%02x%02x", le32dec(&u[0]), le16dec(&u[4]),
	    le16dec(&u[6]), u[8], u[9], u[10], u[11], u[12], u[13], u[14],
	    u[15]);
}

int
mei_uuid_match(const uint8_t *uuid, const char *str)
{
	char buf[40];

	mei_uuid_str(uuid, buf, sizeof(buf));
	return (strcmp(buf, str) == 0);
}

static const struct {
	const char	*uuid;
	const char	*name;
} mei_known_clients[] = {
	{ "3e8d0870-271a-4208-8eb5-9acb9402ae04", "IPTS" },
	{ "8e6a6715-9abc-4043-88ef-9e39c6f63e0f", "MKHI" },
};

static const char *
mei_client_name(const char *uuid)
{
	u_int i;

	for (i = 0; i < nitems(mei_known_clients); i++)
		if (strcmp(uuid, mei_known_clients[i].uuid) == 0)
			return (mei_known_clients[i].name);
	return ("");
}

static int
mei_hbm_enum(struct mei_softc *sc)
{
	uint8_t req[4], map[32];
	struct mei_client *cl;
	char uuid[40];
	int addr, error, n;

	memset(req, 0, sizeof(req));
	req[0] = HBM_ENUM_REQ;
	error = mei_hbm_request(sc, req, sizeof(req), HBM_ENUM_RES_LEN);
	if (error != 0)
		return (error);
	memcpy(map, &sc->hbm_rsp[4], sizeof(map));

	n = 0;
	for (addr = 0; addr < 256; addr++)
		if (map[addr / 8] & (1 << (addr % 8)))
			n++;
	if (n == 0)
		return (0);

	/* M_NOWAIT: the lock is held. */
	sc->clients = malloc(n * sizeof(*sc->clients), M_MEI,
	    M_NOWAIT | M_ZERO);
	if (sc->clients == NULL)
		return (ENOMEM);

	for (addr = 0; addr < 256; addr++) {
		if ((map[addr / 8] & (1 << (addr % 8))) == 0)
			continue;
		memset(req, 0, sizeof(req));
		req[0] = HBM_PROPS_REQ;
		req[1] = addr;
		error = mei_hbm_request(sc, req, sizeof(req),
		    HBM_PROPS_RES_LEN);
		if (error != 0)
			return (error);
		if (sc->hbm_rsp[2] != 0) {
			device_printf(sc->dev, "client %d: status %u\n",
			    addr, sc->hbm_rsp[2]);
			continue;
		}
		cl = &sc->clients[sc->nclients];
		cl->host_addr = ++sc->nclients;
		cl->props.addr = addr;
		memcpy(cl->props.uuid, &sc->hbm_rsp[4], 16);
		cl->props.version = sc->hbm_rsp[20];
		cl->props.max_conn = sc->hbm_rsp[21];
		cl->props.fixed = sc->hbm_rsp[22];
		cl->props.single_recv = sc->hbm_rsp[23] & 1;
		cl->props.max_msg = le32dec(&sc->hbm_rsp[24]);
		/*
		 * The ME does not accept a connection to a client with a
		 * fixed address.  The host sends to it from host address 0,
		 * and neither side uses flow control.
		 */
		if (cl->props.fixed)
			cl->host_addr = 0;

		mei_uuid_str(cl->props.uuid, uuid, sizeof(uuid));
		device_printf(sc->dev, "client %3d: %s v%u conn %u fixed %u "
		    "max_msg %u %s\n", addr, uuid, cl->props.version,
		    cl->props.max_conn, cl->props.fixed, cl->props.max_msg,
		    mei_client_name(uuid));
	}
	return (0);
}

/*
 * Client interface.
 */
static struct mei_client *
mei_cl(device_t child, struct mei_softc **scp)
{
	*scp = device_get_softc(device_get_parent(child));
	return (device_get_ivars(child));
}

const struct mei_client_props *
mei_cl_props(device_t dev)
{
	struct mei_softc *sc;

	return (&mei_cl(dev, &sc)->props);
}

int
mei_cl_connect(device_t dev)
{
	struct mei_softc *sc;
	struct mei_client *cl = mei_cl(dev, &sc);
	uint8_t req[4];
	int error;

	sx_xlock(&sc->hbm_lock);
	mtx_lock(&sc->mtx);
	if (!sc->ready) {
		error = ENXIO;
		goto out;
	}
	if (cl->connected) {
		error = EISCONN;
		goto out;
	}
	cl->tx_credits = 0;
	cl->rx_credit = 0;
	cl->rxq_head = 0;
	cl->rxq_count = 0;
	if (cl->props.fixed) {
		cl->connected = 1;
		error = 0;
		goto out;
	}

	req[0] = HBM_CONNECT_REQ;
	req[1] = cl->props.addr;
	req[2] = cl->host_addr;
	req[3] = 0;
	error = mei_hbm_request(sc, req, sizeof(req), HBM_CONNECT_RES_LEN);
	if (error != 0)
		goto out;
	if (sc->hbm_rsp[3] != 0) {
		device_printf(sc->dev, "client %u: connect status %u\n",
		    cl->props.addr, sc->hbm_rsp[3]);
		error = ECONNREFUSED;
		goto out;
	}
	cl->connected = 1;
	mei_send_flow_control(sc, cl);
out:
	mtx_unlock(&sc->mtx);
	sx_xunlock(&sc->hbm_lock);
	return (error);
}

int
mei_cl_disconnect(device_t dev)
{
	struct mei_softc *sc;
	struct mei_client *cl = mei_cl(dev, &sc);
	uint8_t req[4];
	int error;

	sx_xlock(&sc->hbm_lock);
	mtx_lock(&sc->mtx);
	error = 0;
	if (!cl->connected)
		goto out;
	if (!cl->props.fixed) {
		req[0] = HBM_DISCONNECT_REQ;
		req[1] = cl->props.addr;
		req[2] = cl->host_addr;
		req[3] = 0;
		error = mei_hbm_request(sc, req, sizeof(req),
		    HBM_CONNECT_RES_LEN);
	}
	cl->connected = 0;
	cl->tx_credits = 0;
	cl->rx_credit = 0;
	wakeup(&cl->tx_credits);
	wakeup(&cl->rxq_count);
out:
	mtx_unlock(&sc->mtx);
	sx_xunlock(&sc->hbm_lock);
	return (error);
}

int
mei_cl_send(device_t dev, const void *buf, size_t len, int timo)
{
	struct mei_softc *sc;
	struct mei_client *cl = mei_cl(dev, &sc);
	int error, t;

	if (len > MEI_MAX_MSG || len > cl->props.max_msg)
		return (EMSGSIZE);

	mtx_lock(&sc->mtx);
	for (t = 0; cl->connected && !cl->props.fixed && cl->tx_credits == 0;
	    t += MEI_POLL) {
		if (t >= timo) {
			mtx_unlock(&sc->mtx);
			return (ETIMEDOUT);
		}
		mei_process(sc);
		if (cl->tx_credits > 0)
			break;
		msleep(&cl->tx_credits, &sc->mtx, 0, "meitx", MEI_POLL);
	}
	if (!cl->connected) {
		mtx_unlock(&sc->mtx);
		return (ENOTCONN);
	}
	for (t = 0; t < 10; t++) {
		error = mei_write(sc, cl->props.addr, cl->host_addr, buf, len);
		if (error != EAGAIN)
			break;
		msleep(sc, &sc->mtx, 0, "meiwr", MEI_POLL);
	}
	if (error == 0) {
		if (!cl->props.fixed)
			cl->tx_credits--;
		cl->st_tx++;
	}
	mtx_unlock(&sc->mtx);
	return (error);
}

int
mei_cl_recv(device_t dev, void *buf, size_t maxlen, size_t *len, int timo)
{
	struct mei_softc *sc;
	struct mei_client *cl = mei_cl(dev, &sc);
	struct mei_rx_msg *m;
	int t;

	mtx_lock(&sc->mtx);
	for (t = 0; cl->rxq_count == 0; t += MEI_POLL) {
		if (!cl->connected) {
			mtx_unlock(&sc->mtx);
			return (ENOTCONN);
		}
		if (t >= timo) {
			mtx_unlock(&sc->mtx);
			return (ETIMEDOUT);
		}
		mei_process(sc);
		if (cl->rxq_count > 0)
			break;
		msleep(&cl->rxq_count, &sc->mtx, 0, "meirx", MEI_POLL);
	}
	m = &cl->rxq[cl->rxq_head];
	memcpy(buf, m->data, MIN(maxlen, m->len));
	*len = m->len;
	cl->rxq_head = (cl->rxq_head + 1) % MEI_RXQ_LEN;
	cl->rxq_count--;
	mei_send_flow_control(sc, cl);
	mtx_unlock(&sc->mtx);
	return (0);
}

/*
 * Sysctls.
 */
static int
mei_clients_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct mei_softc *sc = arg1;
	struct sbuf sb;
	struct mei_client *cl;
	char uuid[40];
	int error, i;

	sbuf_new_for_sysctl(&sb, NULL, 256, req);
	mtx_lock(&sc->mtx);
	for (i = 0; i < sc->nclients; i++) {
		cl = &sc->clients[i];
		mei_uuid_str(cl->props.uuid, uuid, sizeof(uuid));
		sbuf_printf(&sb, "\n%3u %s v%u conn %u fixed %u max_msg %u "
		    "%s connected %d tx %lu rx %lu drop %lu",
		    cl->props.addr, uuid, cl->props.version,
		    cl->props.max_conn, cl->props.fixed, cl->props.max_msg,
		    mei_client_name(uuid), cl->connected, cl->st_tx,
		    cl->st_rx, cl->st_rx_drop);
	}
	mtx_unlock(&sc->mtx);
	error = sbuf_finish(&sb);
	sbuf_delete(&sb);
	return (error);
}

static int
mei_regs_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct mei_softc *sc = arg1;
	char buf[96];

	snprintf(buf, sizeof(buf), "H_CSR %#x ME_CSR %#x D0I3C %#x",
	    RD4(sc, MEI_H_CSR), RD4(sc, MEI_ME_CSR),
	    sc->d0i3_supported ? RD4(sc, MEI_H_D0I3C) : 0);
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static void
mei_add_sysctls(struct mei_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *list;

	ctx = device_get_sysctl_ctx(sc->dev);
	list = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "clients",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    mei_clients_sysctl, "A", "ME clients");
	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "regs",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    mei_regs_sysctl, "A", "Interface registers");
	SYSCTL_ADD_U8(ctx, list, OID_AUTO, "hbm_major", CTLFLAG_RD,
	    &sc->hbm_major, 0, "HBM major version");
	SYSCTL_ADD_U8(ctx, list, OID_AUTO, "hbm_minor", CTLFLAG_RD,
	    &sc->hbm_minor, 0, "HBM minor version");
	SYSCTL_ADD_OPAQUE(ctx, list, OID_AUTO, "fw_status", CTLFLAG_RD,
	    sc->hfs, sizeof(sc->hfs), "IU", "Firmware status registers");
	SYSCTL_ADD_INT(ctx, list, OID_AUTO, "msi", CTLFLAG_RD,
	    &sc->msi, 0, "MSI in use");
#define	STAT(name, field, desc)						\
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, name, CTLFLAG_RD,		\
	    &sc->field, desc)
	STAT("intr", st_intr, "Interrupts");
	STAT("rx_msgs", st_rx_msgs, "Messages received");
	STAT("rx_hbm_other", st_rx_hbm_other, "Unexpected HBM messages");
	STAT("rx_client_unknown", st_rx_client_unknown,
	    "Messages for clients that are not connected");
	STAT("rx_errors", st_rx_errors, "Receive errors");
	STAT("me_resets", st_me_resets, "Resets from the ME");
#undef STAT
}

/*
 * Device interface.
 */
static int
mei_probe(device_t dev)
{
	if (pci_get_vendor(dev) != 0x8086 || pci_get_device(dev) != 0x34e4)
		return (ENXIO);
	device_set_desc(dev, "Intel MEI (iTouch)");
	return (BUS_PROBE_DEFAULT);
}

static int
mei_attach(device_t dev)
{
	struct mei_softc *sc = device_get_softc(dev);
	static const int hfs_reg[6] = { MEI_PCI_HFS_1, MEI_PCI_HFS_2,
	    MEI_PCI_HFS_3, MEI_PCI_HFS_4, MEI_PCI_HFS_5, MEI_PCI_HFS_6 };
	device_t child;
	int count, error, i;

	sc->dev = dev;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	sx_init(&sc->hbm_lock, "mei hbm");

	for (i = 0; i < 6; i++)
		sc->hfs[i] = pci_read_config(dev, hfs_reg[i], 4);
	sc->d0i3_supported = (sc->hfs[0] & MEI_HFS_1_D0I3) != 0;
	if (bootverbose)
		device_printf(dev, "FW status %08x %08x %08x %08x %08x "
		    "%08x\n", sc->hfs[0], sc->hfs[1], sc->hfs[2], sc->hfs[3],
		    sc->hfs[4], sc->hfs[5]);

	pci_enable_busmaster(dev);

	sc->mem_rid = PCIR_BAR(0);
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot allocate memory\n");
		error = ENXIO;
		goto fail;
	}

	count = 1;
	if (pci_alloc_msi(dev, &count) == 0) {
		sc->msi = 1;
		sc->irq_rid = 1;
	}
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE | (sc->msi ? 0 : RF_SHAREABLE));
	if (sc->irq == NULL) {
		device_printf(dev, "cannot allocate interrupt\n");
		error = ENXIO;
		goto fail;
	}
	error = bus_setup_intr(dev, sc->irq, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, mei_intr, sc, &sc->intr_cookie);
	if (error != 0) {
		device_printf(dev, "cannot set up interrupt\n");
		goto fail;
	}

	mei_add_sysctls(sc);

	sx_xlock(&sc->hbm_lock);
	mtx_lock(&sc->mtx);
	error = mei_hw_start(sc);
	if (error == 0)
		error = mei_hbm_start(sc);
	if (error == 0)
		error = mei_hbm_enum(sc);
	if (error == 0)
		sc->ready = 1;
	mtx_unlock(&sc->mtx);
	sx_xunlock(&sc->hbm_lock);
	if (error != 0) {
		/* Stay attached, so that the registers can show why. */
		device_printf(dev, "start failed: %d\n", error);
		return (0);
	}
	device_printf(dev, "%d ME clients, %s\n", sc->nclients,
	    sc->msi ? "MSI" : "INTx");

	for (i = 0; i < sc->nclients; i++) {
		child = device_add_child(dev, NULL, DEVICE_UNIT_ANY);
		if (child == NULL)
			continue;
		sc->clients[i].child = child;
		device_set_ivars(child, &sc->clients[i]);
	}
	bus_attach_children(dev);
	return (0);

fail:
	mei_detach(dev);
	return (error);
}

static int
mei_detach(device_t dev)
{
	struct mei_softc *sc = device_get_softc(dev);
	uint32_t hcsr;
	int error;

	error = bus_generic_detach(dev);
	if (error != 0)
		return (error);

	if (sc->mem != NULL) {
		/* Stop the interface: disable interrupts, host not ready. */
		mtx_lock(&sc->mtx);
		sc->ready = 0;
		hcsr = RD4(sc, MEI_H_CSR);
		hcsr &= ~(H_CSR_IE_MASK | CSR_RDY);
		hcsr |= CSR_RST | CSR_IG;
		mei_hcsr_set(sc, hcsr);
		DELAY(100);
		hcsr = RD4(sc, MEI_H_CSR);
		hcsr &= ~(H_CSR_IE_MASK | CSR_RST);
		hcsr |= CSR_IG;
		mei_hcsr_set(sc, hcsr);
		mtx_unlock(&sc->mtx);
	}
	if (sc->intr_cookie != NULL)
		bus_teardown_intr(dev, sc->irq, sc->intr_cookie);
	if (sc->irq != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq);
	if (sc->msi)
		pci_release_msi(dev);
	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid,
		    sc->mem);
	free(sc->clients, M_MEI);
	sx_destroy(&sc->hbm_lock);
	mtx_destroy(&sc->mtx);
	return (0);
}

static int
mei_print_child(device_t dev, device_t child)
{
	struct mei_client *cl = device_get_ivars(child);
	int retval;

	retval = bus_print_child_header(dev, child);
	if (cl != NULL)
		retval += printf(" at client %u", cl->props.addr);
	retval += bus_print_child_footer(dev, child);
	return (retval);
}

static device_method_t mei_methods[] = {
	DEVMETHOD(device_probe,		mei_probe),
	DEVMETHOD(device_attach,	mei_attach),
	DEVMETHOD(device_detach,	mei_detach),

	DEVMETHOD(bus_print_child,	mei_print_child),

	DEVMETHOD_END
};

static driver_t mei_driver = {
	"mei",
	mei_methods,
	sizeof(struct mei_softc),
};

DRIVER_MODULE(mei, pci, mei_driver, 0, 0);
MODULE_DEPEND(mei, pci, 1, 1, 1);
MODULE_VERSION(mei, 1);
