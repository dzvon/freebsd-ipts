/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Dezhi Wu
 *
 * Driver for Intel Precise Touch & Stylus (IPTS).
 *
 * The IPTS touch controller is an ME client.  The host sends commands
 * to it through MEI.  Each command is a 32-bit command code and a
 * payload.  Each response is the command code with bit 31 set, a
 * 32-bit status and a payload.
 *
 * The host gives the controller a set of DMA buffers (the "memory
 * window"): 16 data buffers, 16 feedback buffers, a work queue, a
 * doorbell and a HID-to-ME buffer.  The controller writes a frame into
 * the next data buffer and increments the doorbell.  The host reads
 * the frame and then returns the buffer with a FEEDBACK command.
 *
 * In event mode, the host sends READY_FOR_DATA, and the controller
 * answers when new data is available.  In poll mode, the host reads the
 * doorbell at an interval.
 *
 * The driver is a HID transport: it adds a hidbus(4) child and gives it
 * the HID reports from the data buffers.  The controller runs in
 * single-touch mode, in which the firmware sends the position of one
 * finger in report 0x40.  This report is not in the descriptor of the
 * device, so the driver adds its own descriptor for it.  It changes the
 * report into a one-contact multitouch report, so that hmt(4) can use
 * it.  The pen reports go to hpen(4) without a change.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/sbuf.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/hid/hid.h>

#include "hid_if.h"
#include "mei.h"

#define	IPTS_UUID	"3e8d0870-271a-4208-8eb5-9acb9402ae04"

/* Commands. */
#define	IPTS_CMD_GET_DEVICE_INFO	0x01
#define	IPTS_CMD_SET_MODE		0x02
#define	IPTS_CMD_SET_MEM_WINDOW		0x03
#define	IPTS_CMD_QUIESCE_IO		0x04
#define	IPTS_CMD_READY_FOR_DATA		0x05
#define	IPTS_CMD_FEEDBACK		0x06
#define	IPTS_CMD_CLEAR_MEM_WINDOW	0x07
#define	IPTS_CMD_GET_DESCRIPTOR		0x0f
#define	IPTS_RSP_BIT			0x80000000u

/* Status codes. */
#define	IPTS_STATUS_SUCCESS		0x00
#define	IPTS_STATUS_SENSOR_DISABLED	0x09
#define	IPTS_STATUS_EXPECTED_RESET	0x0b

#define	IPTS_MODE_EVENT			0
#define	IPTS_MODE_POLL			1

#define	IPTS_CMD_MAX_PAYLOAD		320
#define	IPTS_RSP_MAX			(8 + 80)
#define	IPTS_TIMEOUT			(5 * hz)

#define	IPTS_BUFFERS			16
#define	IPTS_WORKQUEUE_SIZE		8192
#define	IPTS_WORKQUEUE_ITEM_SIZE	16

/* Payload of SET_MEM_WINDOW. */
#define	IPTS_MW_DATA_LO(i)		(0 + 4 * (i))
#define	IPTS_MW_DATA_HI(i)		(64 + 4 * (i))
#define	IPTS_MW_WORKQUEUE_LO		128
#define	IPTS_MW_WORKQUEUE_HI		132
#define	IPTS_MW_DOORBELL_LO		136
#define	IPTS_MW_DOORBELL_HI		140
#define	IPTS_MW_FEEDBACK_LO(i)		(144 + 4 * (i))
#define	IPTS_MW_FEEDBACK_HI(i)		(208 + 4 * (i))
#define	IPTS_MW_HID2ME_LO		272
#define	IPTS_MW_HID2ME_HI		276
#define	IPTS_MW_HID2ME_SIZE		280
#define	IPTS_MW_WORKQUEUE_ITEM_SIZE	285
#define	IPTS_MW_WORKQUEUE_SIZE		286
#define	IPTS_MW_LEN			320

/* Header at the start of a data buffer. */
#define	IPTS_DATA_HDR_LEN		64
#define	IPTS_DATA_TYPE_HID		0x03
#define	IPTS_DATA_TYPE_GET_FEATURES	0x04
#define	IPTS_DATA_TYPE_DESCRIPTOR	0x05
#define	IPTS_DATA_TYPES			8
/* Type that marks a data buffer as empty (our own value). */
#define	IPTS_DATA_TYPE_EMPTY		0xffffffffu

/* GET_DESCRIPTOR: the descriptor follows 8 more bytes. */
#define	IPTS_DESC_MAGIC			8
#define	IPTS_DESC_SKIP			8

/*
 * Feedback header: command type (0), payload size (4), buffer index
 * (8), protocol version (12), data type (16), SPI offset (20).  The
 * payload starts at offset 64.
 */
#define	IPTS_FEEDBACK_HDR_LEN		64

/* The HID-to-ME buffer has the index after the last feedback buffer. */
#define	IPTS_HID2ME_BUFFER		IPTS_BUFFERS
#define	IPTS_FEEDBACK_SET_FEATURES	0x01
#define	IPTS_FEEDBACK_GET_FEATURES	0x02
#define	IPTS_FEEDBACK_OUTPUT_REPORT	0x03

/* Response routing: one slot for each command code. */
#define	IPTS_RSP_SLOTS			16
#define	IPTS_RX_POLL			MAX(1, hz / 20)

/* Single-touch report of the firmware: ID, tip, X, Y. */
#define	IPTS_ST_REPORT_ID		0x40
#define	IPTS_ST_REPORT_LEN		6
/* Our one-contact multitouch report and feature report. */
#define	IPTS_MT_REPORT_LEN		8
#define	IPTS_MT_MAX_REPORT_ID		0x41

/* Bus type for HID devices on MEI (as Linux uses). */
#define	IPTS_HID_BUS			0x44

/*
 * Descriptor for the single-touch report, written as a touchscreen with
 * one contact:
 *   0x40: tip (1 bit), padding (7 bits), contact ID (8 bits),
 *         X (16 bits), Y (16 bits), contact count (8 bits)
 *   0x41: feature, contact count maximum (8 bits)
 * The physical size is the size of the Surface Pro 7 screen.
 */
static const uint8_t ipts_st_desc[] = {
	0x05, 0x0d,		/* Usage Page (Digitizers) */
	0x09, 0x04,		/* Usage (Touch Screen) */
	0xa1, 0x01,		/* Collection (Application) */
	0x85, IPTS_ST_REPORT_ID, /*  Report ID */
	0x09, 0x22,		/*   Usage (Finger) */
	0xa1, 0x02,		/*   Collection (Logical) */
	0x09, 0x42,		/*     Usage (Tip Switch) */
	0x15, 0x00,		/*     Logical Minimum (0) */
	0x25, 0x01,		/*     Logical Maximum (1) */
	0x75, 0x01,		/*     Report Size (1) */
	0x95, 0x01,		/*     Report Count (1) */
	0x81, 0x02,		/*     Input (Data, Variable, Absolute) */
	0x95, 0x07,		/*     Report Count (7) */
	0x81, 0x03,		/*     Input (Constant, Variable) */
	0x09, 0x51,		/*     Usage (Contact Identifier) */
	0x25, 0x0f,		/*     Logical Maximum (15) */
	0x75, 0x08,		/*     Report Size (8) */
	0x95, 0x01,		/*     Report Count (1) */
	0x81, 0x02,		/*     Input (Data, Variable, Absolute) */
	0x05, 0x01,		/*     Usage Page (Generic Desktop) */
	0x09, 0x30,		/*     Usage (X) */
	0x75, 0x10,		/*     Report Size (16) */
	0x55, 0x0e,		/*     Unit Exponent (-2) */
	0x65, 0x11,		/*     Unit (Centimeter) */
	0x35, 0x00,		/*     Physical Minimum (0) */
	0x46, 0x26, 0x0a,	/*     Physical Maximum (2598) */
	0x26, 0xff, 0x7f,	/*     Logical Maximum (32767) */
	0x81, 0x02,		/*     Input (Data, Variable, Absolute) */
	0x09, 0x31,		/*     Usage (Y) */
	0x46, 0xc4, 0x06,	/*     Physical Maximum (1732) */
	0x81, 0x02,		/*     Input (Data, Variable, Absolute) */
	0xc0,			/*   End Collection */
	0x05, 0x0d,		/*   Usage Page (Digitizers) */
	0x55, 0x00,		/*   Unit Exponent (0) */
	0x65, 0x00,		/*   Unit (None) */
	0x45, 0x00,		/*   Physical Maximum (0) */
	0x09, 0x54,		/*   Usage (Contact Count) */
	0x25, 0x0a,		/*   Logical Maximum (10) */
	0x75, 0x08,		/*   Report Size (8) */
	0x81, 0x02,		/*   Input (Data, Variable, Absolute) */
	0x85, IPTS_MT_MAX_REPORT_ID, /* Report ID */
	0x09, 0x55,		/*   Usage (Contact Count Maximum) */
	0xb1, 0x02,		/*   Feature (Data, Variable, Absolute) */
	0xc0,			/* End Collection */
};

static MALLOC_DEFINE(M_IPTS, "ipts", "IPTS driver");

static int ipts_mode = IPTS_MODE_EVENT;
TUNABLE_INT("hw.ipts.mode", &ipts_mode);

struct ipts_dma {
	bus_dma_tag_t	tag;
	bus_dmamap_t	map;
	void		*vaddr;
	bus_addr_t	paddr;
	size_t		size;
	int		loaded;
};

struct ipts_rsp {
	int		valid;
	size_t		len;
	uint8_t		data[IPTS_RSP_MAX];
};

struct ipts_softc {
	device_t	dev;
	int		connected;
	int		mode;

	uint8_t		info[80];
	size_t		info_len;
	uint16_t	vendor;
	uint16_t	product;
	uint32_t	data_size;
	uint32_t	feedback_size;
	uint8_t		max_contacts;
	uint8_t		intf_eds;

	struct ipts_dma	data[IPTS_BUFFERS];
	struct ipts_dma	feedback[IPTS_BUFFERS];
	struct ipts_dma	workqueue;
	struct ipts_dma	doorbell;
	struct ipts_dma	hid2me;
	struct ipts_dma	descriptor;
	int		mem_window_set;
	int		keep_dma;	/* do not free: device can still write */

	uint8_t		*hid_desc;
	size_t		hid_desc_len;

	/* HID transport. */
	device_t		hidbus;
	struct hid_device_info	hw;
	uint8_t			*rdesc;		/* our part + device part */
	size_t			rdesc_len;
	hid_intr_t		*intr;
	void			*intr_ctx;
	int			intr_started;
	int			in_intr;
	u_long			st_hid_reports;

	struct proc	*proc;
	volatile int	stop;
	struct mtx	mtx;
	uint32_t	last_doorbell;
	u_int		next_buffer;	/* event mode */

	u_long		st_frames;
	u_long		st_types[IPTS_DATA_TYPES];
	u_long		st_errors;
	uint32_t	last_type;
	uint32_t	last_size;
	uint8_t		last_frame[256];
	int		dump;
	u_long		st_ready;
	uint8_t		ready_rsp[80];

	/* Response routing, protected by mtx. */
	struct ipts_rsp	rsp[IPTS_RSP_SLOTS];
	int		rx_reader;
	u_long		st_rsp_unexpected;
	struct sx	cmd_lock;	/* one command with response */

	/* Feature reports through the HID-to-ME buffer. */
	struct sx	feat_lock;
	int		feat_wait;	/* protected by mtx */
	int		feat_done;
	uint8_t		*feat_buf;
	size_t		feat_len;

	u_long		st_ids[256];	/* input reports by ID */
};

static void	ipts_hid_input(struct ipts_softc *sc, const uint8_t *rep,
		    size_t len);

static int
ipts_cmd_send(struct ipts_softc *sc, uint32_t code, const void *pld,
    size_t plen)
{
	uint8_t buf[4 + IPTS_CMD_MAX_PAYLOAD];

	if (plen > IPTS_CMD_MAX_PAYLOAD)
		return (EINVAL);
	le32enc(buf, code);
	if (plen > 0)
		memcpy(&buf[4], pld, plen);
	return (mei_cl_send(sc->dev, buf, 4 + plen, IPTS_TIMEOUT));
}

/* Forget an old response before a new command is sent. */
static void
ipts_rsp_clear(struct ipts_softc *sc, uint32_t code)
{
	mtx_lock(&sc->mtx);
	sc->rsp[code % IPTS_RSP_SLOTS].valid = 0;
	mtx_unlock(&sc->mtx);
}

/*
 * Wait for the response to a command.  More than one thread can wait
 * for responses at the same time (the receive thread waits for
 * READY_FOR_DATA while another thread sends FEEDBACK).  One of the
 * waiting threads reads from MEI and puts each response into the slot
 * for its command code.
 */
static int
ipts_cmd_recv(struct ipts_softc *sc, uint32_t code, void *rsp,
    size_t rsp_max, size_t *rsp_len, uint32_t *status, int timo)
{
	struct ipts_rsp *r = &sc->rsp[code % IPTS_RSP_SLOTS];
	uint8_t rbuf[IPTS_RSP_MAX];
	uint32_t c;
	size_t len;
	int error, t;

	mtx_lock(&sc->mtx);
	for (t = 0; !r->valid; t += IPTS_RX_POLL) {
		if (t >= timo) {
			mtx_unlock(&sc->mtx);
			return (ETIMEDOUT);
		}
		if (sc->rx_reader) {
			msleep(&sc->rsp, &sc->mtx, 0, "iptsrsp", IPTS_RX_POLL);
			continue;
		}
		sc->rx_reader = 1;
		mtx_unlock(&sc->mtx);
		error = mei_cl_recv(sc->dev, rbuf, sizeof(rbuf), &len,
		    IPTS_RX_POLL);
		mtx_lock(&sc->mtx);
		sc->rx_reader = 0;
		wakeup(&sc->rsp);
		if (error == ETIMEDOUT)
			continue;
		if (error != 0) {
			mtx_unlock(&sc->mtx);
			return (error);
		}
		c = len >= 8 ? le32dec(rbuf) : 0;
		if ((c & IPTS_RSP_BIT) == 0 ||
		    (c & ~IPTS_RSP_BIT) >= IPTS_RSP_SLOTS) {
			sc->st_rsp_unexpected++;
			continue;
		}
		c &= ~IPTS_RSP_BIT;
		sc->rsp[c].len = MIN(len, sizeof(rbuf));
		memcpy(sc->rsp[c].data, rbuf, sc->rsp[c].len);
		sc->rsp[c].valid = 1;
	}
	r->valid = 0;
	*status = le32dec(&r->data[4]);
	len = r->len - 8;
	if (rsp != NULL)
		memcpy(rsp, &r->data[8], MIN(len, rsp_max));
	if (rsp_len != NULL)
		*rsp_len = len;
	mtx_unlock(&sc->mtx);
	return (0);
}

/* Send a command, wait for the response, and check the status. */
static int
ipts_cmd(struct ipts_softc *sc, uint32_t code, const void *pld, size_t plen,
    void *rsp, size_t rsp_max, size_t *rsp_len)
{
	uint32_t status;
	int error;

	sx_xlock(&sc->cmd_lock);
	ipts_rsp_clear(sc, code);
	error = ipts_cmd_send(sc, code, pld, plen);
	if (error == 0)
		error = ipts_cmd_recv(sc, code, rsp, rsp_max, rsp_len,
		    &status, IPTS_TIMEOUT);
	sx_xunlock(&sc->cmd_lock);
	if (error != 0) {
		device_printf(sc->dev, "command %#x failed: %d\n", code,
		    error);
		return (error);
	}
	if (status == IPTS_STATUS_EXPECTED_RESET) {
		device_printf(sc->dev, "command %#x: sensor reset\n", code);
		return (0);
	}
	if (status != IPTS_STATUS_SUCCESS) {
		device_printf(sc->dev, "command %#x: status %#x\n", code,
		    status);
		return (EIO);
	}
	return (0);
}

/*
 * DMA buffers.
 */
static void
ipts_dma_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	if (error == 0 && nseg == 1)
		*(bus_addr_t *)arg = segs[0].ds_addr;
}

static int
ipts_dma_alloc(struct ipts_softc *sc, struct ipts_dma *d, size_t size)
{
	int error;

	size = round_page(size);
	d->size = size;
	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), PAGE_SIZE, 0,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL, size, 1, size,
	    0, NULL, NULL, &d->tag);
	if (error != 0)
		return (error);
	error = bus_dmamem_alloc(d->tag, &d->vaddr,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT, &d->map);
	if (error != 0)
		return (error);
	d->paddr = 0;
	error = bus_dmamap_load(d->tag, d->map, d->vaddr, size, ipts_dma_cb,
	    &d->paddr, BUS_DMA_NOWAIT);
	if (error != 0 || d->paddr == 0)
		return (error != 0 ? error : ENOMEM);
	d->loaded = 1;
	return (0);
}

static void
ipts_dma_free(struct ipts_dma *d)
{
	if (d->loaded)
		bus_dmamap_unload(d->tag, d->map);
	if (d->vaddr != NULL)
		bus_dmamem_free(d->tag, d->vaddr, d->map);
	if (d->tag != NULL)
		bus_dma_tag_destroy(d->tag);
	memset(d, 0, sizeof(*d));
}

static int
ipts_alloc_buffers(struct ipts_softc *sc)
{
	int error, i;

	for (i = 0; i < IPTS_BUFFERS; i++) {
		if ((error = ipts_dma_alloc(sc, &sc->data[i],
		    sc->data_size)) != 0)
			return (error);
		if ((error = ipts_dma_alloc(sc, &sc->feedback[i],
		    sc->feedback_size)) != 0)
			return (error);
	}
	if ((error = ipts_dma_alloc(sc, &sc->workqueue,
	    IPTS_WORKQUEUE_SIZE)) != 0)
		return (error);
	if ((error = ipts_dma_alloc(sc, &sc->doorbell, sizeof(uint32_t))) != 0)
		return (error);
	if ((error = ipts_dma_alloc(sc, &sc->hid2me, sc->feedback_size)) != 0)
		return (error);
	return (ipts_dma_alloc(sc, &sc->descriptor,
	    sc->data_size + IPTS_DESC_SKIP));
}

static void
ipts_free_buffers(struct ipts_softc *sc)
{
	int i;

	for (i = 0; i < IPTS_BUFFERS; i++) {
		ipts_dma_free(&sc->data[i]);
		ipts_dma_free(&sc->feedback[i]);
	}
	ipts_dma_free(&sc->workqueue);
	ipts_dma_free(&sc->doorbell);
	ipts_dma_free(&sc->hid2me);
	ipts_dma_free(&sc->descriptor);
}

static uint32_t
ipts_read_doorbell(struct ipts_softc *sc)
{
	bus_dmamap_sync(sc->doorbell.tag, sc->doorbell.map,
	    BUS_DMASYNC_POSTREAD);
	return (le32toh(*(volatile uint32_t *)sc->doorbell.vaddr));
}

/*
 * Commands.
 */
static int
ipts_get_device_info(struct ipts_softc *sc)
{
	int error;

	error = ipts_cmd(sc, IPTS_CMD_GET_DEVICE_INFO, NULL, 0, sc->info,
	    sizeof(sc->info), &sc->info_len);
	if (error != 0)
		return (error);
	if (sc->info_len < 44)
		return (EIO);

	sc->vendor = le16dec(&sc->info[0]);
	sc->product = le16dec(&sc->info[2]);
	sc->data_size = le32dec(&sc->info[12]);
	sc->feedback_size = le32dec(&sc->info[16]);
	sc->max_contacts = sc->info[24];
	sc->intf_eds = sc->info[32];

	device_printf(sc->dev, "touch controller %04x:%04x hw %#x fw %#x, "
	    "%u contacts, EDS %u, buffers %u/%u\n", sc->vendor, sc->product,
	    le32dec(&sc->info[4]), le32dec(&sc->info[8]), sc->max_contacts,
	    sc->intf_eds, sc->data_size, sc->feedback_size);
	if (sc->data_size == 0 || sc->data_size > 1024 * 1024 ||
	    sc->feedback_size == 0 || sc->feedback_size > 1024 * 1024)
		return (EIO);
	return (0);
}

static int
ipts_set_mode(struct ipts_softc *sc)
{
	uint8_t pld[16];

	memset(pld, 0, sizeof(pld));
	le32enc(pld, sc->mode);
	return (ipts_cmd(sc, IPTS_CMD_SET_MODE, pld, sizeof(pld), NULL, 0,
	    NULL));
}

static int
ipts_set_mem_window(struct ipts_softc *sc)
{
	uint8_t pld[IPTS_MW_LEN];
	int error, i;

	memset(pld, 0, sizeof(pld));
	for (i = 0; i < IPTS_BUFFERS; i++) {
		le32enc(&pld[IPTS_MW_DATA_LO(i)], sc->data[i].paddr);
		le32enc(&pld[IPTS_MW_DATA_HI(i)],
		    (uint64_t)sc->data[i].paddr >> 32);
		le32enc(&pld[IPTS_MW_FEEDBACK_LO(i)], sc->feedback[i].paddr);
		le32enc(&pld[IPTS_MW_FEEDBACK_HI(i)],
		    (uint64_t)sc->feedback[i].paddr >> 32);
	}
	le32enc(&pld[IPTS_MW_WORKQUEUE_LO], sc->workqueue.paddr);
	le32enc(&pld[IPTS_MW_WORKQUEUE_HI],
	    (uint64_t)sc->workqueue.paddr >> 32);
	le32enc(&pld[IPTS_MW_DOORBELL_LO], sc->doorbell.paddr);
	le32enc(&pld[IPTS_MW_DOORBELL_HI], (uint64_t)sc->doorbell.paddr >> 32);
	le32enc(&pld[IPTS_MW_HID2ME_LO], sc->hid2me.paddr);
	le32enc(&pld[IPTS_MW_HID2ME_HI], (uint64_t)sc->hid2me.paddr >> 32);
	le32enc(&pld[IPTS_MW_HID2ME_SIZE], sc->feedback_size);
	pld[IPTS_MW_WORKQUEUE_ITEM_SIZE] = IPTS_WORKQUEUE_ITEM_SIZE;
	le16enc(&pld[IPTS_MW_WORKQUEUE_SIZE], IPTS_WORKQUEUE_SIZE);

	for (i = 0; i < IPTS_BUFFERS; i++) {
		le32enc(sc->data[i].vaddr, IPTS_DATA_TYPE_EMPTY);
		bus_dmamap_sync(sc->data[i].tag, sc->data[i].map,
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	}

	/* From here on, the device can write to the buffers. */
	sc->mem_window_set = 1;
	error = ipts_cmd(sc, IPTS_CMD_SET_MEM_WINDOW, pld, sizeof(pld), NULL,
	    0, NULL);
	return (error);
}

static int
ipts_get_descriptor(struct ipts_softc *sc)
{
	uint8_t pld[24], *p;
	uint32_t type, size;
	int error;

	memset(sc->descriptor.vaddr, 0, sc->descriptor.size);
	memset(pld, 0, sizeof(pld));
	le32enc(&pld[0], sc->descriptor.paddr);
	le32enc(&pld[4], (uint64_t)sc->descriptor.paddr >> 32);
	le32enc(&pld[8], IPTS_DESC_MAGIC);
	error = ipts_cmd(sc, IPTS_CMD_GET_DESCRIPTOR, pld, sizeof(pld), NULL,
	    0, NULL);
	if (error != 0)
		return (error);

	bus_dmamap_sync(sc->descriptor.tag, sc->descriptor.map,
	    BUS_DMASYNC_POSTREAD);
	p = sc->descriptor.vaddr;
	type = le32dec(&p[0]);
	size = le32dec(&p[4]);
	if (type != IPTS_DATA_TYPE_DESCRIPTOR || size <= IPTS_DESC_SKIP ||
	    IPTS_DATA_HDR_LEN + size > sc->descriptor.size) {
		device_printf(sc->dev, "no HID descriptor (type %u size %u)\n",
		    type, size);
		return (ENXIO);
	}
	sc->hid_desc_len = size - IPTS_DESC_SKIP;
	sc->hid_desc = malloc(sc->hid_desc_len, M_IPTS, M_WAITOK);
	memcpy(sc->hid_desc, &p[IPTS_DATA_HDR_LEN + IPTS_DESC_SKIP],
	    sc->hid_desc_len);
	device_printf(sc->dev, "HID descriptor: %zu bytes\n",
	    sc->hid_desc_len);
	return (0);
}

/* Return a data buffer to the device. */
static int
ipts_feedback(struct ipts_softc *sc, u_int buffer)
{
	struct ipts_dma *fb = &sc->feedback[buffer];
	uint8_t pld[16];

	memset(fb->vaddr, 0, IPTS_FEEDBACK_HDR_LEN);
	le32enc((uint8_t *)fb->vaddr + 8, buffer);
	bus_dmamap_sync(fb->tag, fb->map, BUS_DMASYNC_PREWRITE);

	memset(pld, 0, sizeof(pld));
	pld[0] = buffer;
	return (ipts_cmd(sc, IPTS_CMD_FEEDBACK, pld, sizeof(pld), NULL, 0,
	    NULL));
}

/*
 * Send a report to the device through the HID-to-ME buffer.  The
 * caller holds feat_lock.
 */
static int
ipts_hid2me(struct ipts_softc *sc, uint32_t type, const void *data,
    size_t len)
{
	struct ipts_dma *d = &sc->hid2me;
	uint8_t *p = d->vaddr, pld[16];

	sx_assert(&sc->feat_lock, SA_XLOCKED);
	if (p == NULL || !sc->mem_window_set)
		return (ENXIO);
	if (IPTS_FEEDBACK_HDR_LEN + len > sc->feedback_size)
		return (EMSGSIZE);
	memset(p, 0, IPTS_FEEDBACK_HDR_LEN + len);
	le32enc(&p[4], len);
	le32enc(&p[8], IPTS_HID2ME_BUFFER);
	le32enc(&p[16], type);
	memcpy(&p[IPTS_FEEDBACK_HDR_LEN], data, len);
	bus_dmamap_sync(d->tag, d->map, BUS_DMASYNC_PREWRITE);

	memset(pld, 0, sizeof(pld));
	pld[0] = IPTS_HID2ME_BUFFER;
	return (ipts_cmd(sc, IPTS_CMD_FEEDBACK, pld, sizeof(pld), NULL, 0,
	    NULL));
}

/*
 * HID transport.
 */

/* Give an input report to hidbus.  No lock is held. */
static void
ipts_hid_input(struct ipts_softc *sc, const uint8_t *rep, size_t len)
{
	uint8_t mt[IPTS_MT_REPORT_LEN];
	hid_intr_t *intr;
	void *ctx;

	sc->st_ids[rep[0]]++;
	if (rep[0] == IPTS_ST_REPORT_ID) {
		if (len < IPTS_ST_REPORT_LEN)
			return;
		mt[0] = IPTS_ST_REPORT_ID;
		mt[1] = rep[1] & 0x01;		/* tip */
		mt[2] = 0;			/* contact ID */
		mt[3] = rep[2];			/* X */
		mt[4] = rep[3];
		mt[5] = rep[4];			/* Y */
		mt[6] = rep[5];
		mt[7] = 1;			/* contact count */
		rep = mt;
		len = sizeof(mt);
	}

	mtx_lock(&sc->mtx);
	if (!sc->intr_started || sc->intr == NULL) {
		mtx_unlock(&sc->mtx);
		return;
	}
	intr = sc->intr;
	ctx = sc->intr_ctx;
	sc->in_intr++;
	sc->st_hid_reports++;
	mtx_unlock(&sc->mtx);

	intr(ctx, __DECONST(uint8_t *, rep), len);

	mtx_lock(&sc->mtx);
	if (--sc->in_intr == 0)
		wakeup(&sc->in_intr);
	mtx_unlock(&sc->mtx);
}

static void
ipts_hid_intr_setup(device_t dev, device_t child __unused, hid_intr_t intr,
    void *context, struct hid_rdesc_info *rdesc)
{
	struct ipts_softc *sc = device_get_softc(dev);

	rdesc->rdsize = rdesc->isize;
	rdesc->grsize = rdesc->srsize = rdesc->wrsize =
	    sc->feedback_size - IPTS_FEEDBACK_HDR_LEN;
	mtx_lock(&sc->mtx);
	sc->intr = intr;
	sc->intr_ctx = context;
	mtx_unlock(&sc->mtx);
}

static void
ipts_hid_intr_unsetup(device_t dev, device_t child __unused)
{
	struct ipts_softc *sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);
	sc->intr_started = 0;
	sc->intr = NULL;
	while (sc->in_intr > 0)
		msleep(&sc->in_intr, &sc->mtx, 0, "iptsint", hz);
	mtx_unlock(&sc->mtx);
}

static int
ipts_hid_intr_start(device_t dev, device_t child __unused)
{
	struct ipts_softc *sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);
	sc->intr_started = 1;
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
ipts_hid_intr_stop(device_t dev, device_t child __unused)
{
	struct ipts_softc *sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);
	sc->intr_started = 0;
	mtx_unlock(&sc->mtx);
	return (0);
}

static void
ipts_hid_intr_poll(device_t dev __unused, device_t child __unused)
{
}

static int
ipts_hid_get_rdesc(device_t dev, device_t child __unused, void *buf,
    hid_size_t len)
{
	struct ipts_softc *sc = device_get_softc(dev);

	if (len < sc->rdesc_len)
		return (EINVAL);
	memcpy(buf, sc->rdesc, sc->rdesc_len);
	return (0);
}

static int
ipts_hid_read(device_t dev __unused, device_t child __unused,
    void *buf __unused, hid_size_t maxlen __unused,
    hid_size_t *actlen __unused)
{
	return (ENOTSUP);
}

static int
ipts_hid_write(device_t dev, device_t child __unused, const void *buf,
    hid_size_t len)
{
	struct ipts_softc *sc = device_get_softc(dev);
	int error;

	sx_xlock(&sc->feat_lock);
	error = ipts_hid2me(sc, IPTS_FEEDBACK_OUTPUT_REPORT, buf, len);
	sx_xunlock(&sc->feat_lock);
	return (error);
}

static int
ipts_hid_get_report(device_t dev, device_t child __unused, void *buf,
    hid_size_t maxlen, hid_size_t *actlen, uint8_t type, uint8_t id)
{
	struct ipts_softc *sc = device_get_softc(dev);
	uint8_t *p = buf;
	size_t len;
	int error, t;

	if (type != HID_FEATURE_REPORT || maxlen < 1)
		return (ENOTSUP);

	/* Our own feature report. */
	if (id == IPTS_MT_MAX_REPORT_ID) {
		if (maxlen < 2)
			return (EINVAL);
		p[0] = IPTS_MT_MAX_REPORT_ID;
		p[1] = 1;
		if (actlen != NULL)
			*actlen = 2;
		return (0);
	}

	/*
	 * Ask the device through the HID-to-ME buffer.  The report comes
	 * back in a data buffer of type GET_FEATURES.
	 */
	if (sc->feat_buf == NULL)
		return (ENXIO);
	len = MIN(maxlen, sc->feedback_size - IPTS_FEEDBACK_HDR_LEN);
	sx_xlock(&sc->feat_lock);
	mtx_lock(&sc->mtx);
	sc->feat_wait = 1;
	sc->feat_done = 0;
	mtx_unlock(&sc->mtx);

	memset(p, 0, len);
	p[0] = id;
	error = ipts_hid2me(sc, IPTS_FEEDBACK_GET_FEATURES, p, len);

	mtx_lock(&sc->mtx);
	for (t = 0; error == 0 && !sc->feat_done; t += hz / 10) {
		if (t >= IPTS_TIMEOUT) {
			error = ETIMEDOUT;
			break;
		}
		msleep(&sc->feat_done, &sc->mtx, 0, "iptsft", hz / 10);
	}
	if (error == 0) {
		len = MIN(sc->feat_len, maxlen);
		memcpy(p, sc->feat_buf, len);
		if (actlen != NULL)
			*actlen = len;
	}
	sc->feat_wait = 0;
	mtx_unlock(&sc->mtx);
	sx_xunlock(&sc->feat_lock);
	return (error);
}

static int
ipts_hid_set_report(device_t dev, device_t child __unused, const void *buf,
    hid_size_t len, uint8_t type, uint8_t id __unused)
{
	struct ipts_softc *sc = device_get_softc(dev);
	uint32_t ftype;
	int error;

	if (type == HID_FEATURE_REPORT)
		ftype = IPTS_FEEDBACK_SET_FEATURES;
	else if (type == HID_OUTPUT_REPORT)
		ftype = IPTS_FEEDBACK_OUTPUT_REPORT;
	else
		return (ENOTSUP);
	sx_xlock(&sc->feat_lock);
	error = ipts_hid2me(sc, ftype, buf, len);
	sx_xunlock(&sc->feat_lock);
	return (error);
}

static int
ipts_hid_set_idle(device_t dev __unused, device_t child __unused,
    uint16_t duration __unused, uint8_t id __unused)
{
	return (0);
}

static int
ipts_hid_set_protocol(device_t dev __unused, device_t child __unused,
    uint16_t protocol __unused)
{
	return (0);
}

static int
ipts_hid_ioctl(device_t dev __unused, device_t child __unused,
    unsigned long cmd __unused, uintptr_t data __unused)
{
	return (ENOTTY);
}

/* Make the report descriptor and add the hidbus child. */
static void
ipts_hid_attach(struct ipts_softc *sc)
{
	sc->rdesc_len = sizeof(ipts_st_desc) + sc->hid_desc_len;
	sc->rdesc = malloc(sc->rdesc_len, M_IPTS, M_WAITOK);
	memcpy(sc->rdesc, ipts_st_desc, sizeof(ipts_st_desc));
	if (sc->hid_desc_len > 0)
		memcpy(sc->rdesc + sizeof(ipts_st_desc), sc->hid_desc,
		    sc->hid_desc_len);

	strlcpy(sc->hw.name, "Intel Precise Touch & Stylus",
	    sizeof(sc->hw.name));
	sc->hw.idBus = IPTS_HID_BUS;
	sc->hw.idVendor = sc->vendor;
	sc->hw.idProduct = sc->product;
	sc->hw.idVersion = le32dec(&sc->info[4]);
	sc->hw.rdescsize = sc->rdesc_len;

	sc->hidbus = device_add_child(sc->dev, "hidbus", DEVICE_UNIT_ANY);
	if (sc->hidbus == NULL) {
		device_printf(sc->dev, "cannot add hidbus\n");
		return;
	}
	device_set_ivars(sc->hidbus, &sc->hw);
	bus_attach_children(sc->dev);
}

/*
 * Receive loop.
 */
static void
ipts_handle_buffer(struct ipts_softc *sc, u_int buffer)
{
	struct ipts_dma *d = &sc->data[buffer];
	uint8_t *p = d->vaddr;
	uint32_t type, size;
	int i;

	bus_dmamap_sync(d->tag, d->map, BUS_DMASYNC_POSTREAD);
	type = le32dec(&p[0]);
	size = le32dec(&p[4]);

	mtx_lock(&sc->mtx);
	sc->st_frames++;
	sc->st_types[MIN(type, IPTS_DATA_TYPES - 1)]++;
	sc->last_type = type;
	sc->last_size = size;
	memcpy(sc->last_frame, p, sizeof(sc->last_frame));
	mtx_unlock(&sc->mtx);

	if (type == IPTS_DATA_TYPE_HID && size > 0 &&
	    IPTS_DATA_HDR_LEN + size <= sc->data_size)
		ipts_hid_input(sc, &p[IPTS_DATA_HDR_LEN], size);

	if (type == IPTS_DATA_TYPE_GET_FEATURES &&
	    IPTS_DATA_HDR_LEN + size <= sc->data_size) {
		mtx_lock(&sc->mtx);
		if (sc->feat_wait && !sc->feat_done) {
			memcpy(sc->feat_buf, &p[IPTS_DATA_HDR_LEN], size);
			sc->feat_len = size;
			sc->feat_done = 1;
			wakeup(&sc->feat_done);
		}
		mtx_unlock(&sc->mtx);
	}

	if (sc->dump > 0) {
		sc->dump--;
		printf("%s: buffer %u type %u size %u:", device_get_nameunit(
		    sc->dev), buffer, type, size);
		for (i = 0; i < 96; i++)
			printf("%s%02x", i % 16 == 0 ? "\n  " : " ", p[i]);
		printf("\n");
	}
	bus_dmamap_sync(d->tag, d->map, BUS_DMASYNC_PREREAD);
}

/* Handle all buffers up to the doorbell. */
static int
ipts_handle_doorbell(struct ipts_softc *sc)
{
	uint32_t doorbell;
	u_int buffer;
	int error;

	doorbell = ipts_read_doorbell(sc);
	/* The device can be far ahead; handle at most all buffers once. */
	if (doorbell - sc->last_doorbell > IPTS_BUFFERS)
		sc->last_doorbell = doorbell - IPTS_BUFFERS;
	while (sc->last_doorbell != doorbell) {
		buffer = sc->last_doorbell % IPTS_BUFFERS;
		ipts_handle_buffer(sc, buffer);
		error = ipts_feedback(sc, buffer);
		if (error != 0)
			return (error);
		sc->last_doorbell++;
	}
	return (0);
}

/*
 * Event mode: the device does not use the doorbell.  It fills the data
 * buffers in order.  Handle each full buffer, mark it empty and return
 * it to the device.
 */
static int
ipts_handle_ready(struct ipts_softc *sc)
{
	struct ipts_dma *d;
	u_int buffer, i;
	int error;

	for (i = 0; i < IPTS_BUFFERS; i++) {
		buffer = sc->next_buffer % IPTS_BUFFERS;
		d = &sc->data[buffer];
		bus_dmamap_sync(d->tag, d->map, BUS_DMASYNC_POSTREAD);
		if (le32dec(d->vaddr) == IPTS_DATA_TYPE_EMPTY)
			break;
		ipts_handle_buffer(sc, buffer);
		le32enc(d->vaddr, IPTS_DATA_TYPE_EMPTY);
		bus_dmamap_sync(d->tag, d->map,
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
		error = ipts_feedback(sc, buffer);
		if (error != 0)
			return (error);
		sc->next_buffer++;
	}
	return (0);
}

static void
ipts_thread(void *arg)
{
	struct ipts_softc *sc = arg;
	uint32_t status;
	int error, waiting;

	waiting = 0;
	while (!sc->stop) {
		if (sc->mode == IPTS_MODE_POLL) {
			if (ipts_handle_doorbell(sc) != 0)
				sc->st_errors++;
			pause("iptspl", MAX(1, hz / 100));
			continue;
		}

		/* Event mode. */
		if (!waiting) {
			ipts_rsp_clear(sc, IPTS_CMD_READY_FOR_DATA);
			error = ipts_cmd_send(sc, IPTS_CMD_READY_FOR_DATA,
			    NULL, 0);
			if (error != 0) {
				sc->st_errors++;
				pause("iptser", hz);
				continue;
			}
			waiting = 1;
		}
		error = ipts_cmd_recv(sc, IPTS_CMD_READY_FOR_DATA,
		    sc->ready_rsp, sizeof(sc->ready_rsp), NULL, &status, hz);
		if (error == ETIMEDOUT)
			continue;
		waiting = 0;
		if (error != 0 || status != IPTS_STATUS_SUCCESS) {
			sc->st_errors++;
			if (status == IPTS_STATUS_SENSOR_DISABLED)
				pause("iptsdis", hz);
			continue;
		}
		sc->st_ready++;
		if (ipts_handle_ready(sc) != 0)
			sc->st_errors++;
	}

	mtx_lock(&sc->mtx);
	sc->proc = NULL;
	wakeup(&sc->proc);
	mtx_unlock(&sc->mtx);
	kproc_exit(0);
}

/*
 * Start and stop.
 */
static int
ipts_start(struct ipts_softc *sc)
{
	int error;

	if ((error = ipts_get_device_info(sc)) != 0)
		return (error);
	sc->feat_buf = malloc(sc->data_size, M_IPTS, M_WAITOK | M_ZERO);
	if ((error = ipts_alloc_buffers(sc)) != 0) {
		device_printf(sc->dev, "cannot allocate DMA buffers: %d\n",
		    error);
		return (error);
	}
	if ((error = ipts_set_mode(sc)) != 0)
		return (error);
	if ((error = ipts_set_mem_window(sc)) != 0)
		return (error);
	if (sc->intf_eds >= 2)
		(void)ipts_get_descriptor(sc);

	sc->last_doorbell = ipts_read_doorbell(sc);
	return (kproc_create(ipts_thread, sc, &sc->proc, 0, 0, "%s",
	    device_get_nameunit(sc->dev)));
}

static void
ipts_stop(struct ipts_softc *sc)
{
	uint8_t pld[12];
	int error;

	/* Stop the receive thread. */
	mtx_lock(&sc->mtx);
	sc->stop = 1;
	while (sc->proc != NULL)
		msleep(&sc->proc, &sc->mtx, 0, "iptsstp", hz);
	mtx_unlock(&sc->mtx);

	if (!sc->mem_window_set)
		return;

	/*
	 * Tell the device to stop all DMA before we free the buffers.  The
	 * response to an outstanding READY_FOR_DATA can come first, and
	 * ipts_cmd skips it.
	 */
	memset(pld, 0, sizeof(pld));
	(void)ipts_cmd(sc, IPTS_CMD_QUIESCE_IO, pld, sizeof(pld), NULL, 0,
	    NULL);
	error = ipts_cmd(sc, IPTS_CMD_CLEAR_MEM_WINDOW, NULL, 0, NULL, 0,
	    NULL);
	if (error != 0) {
		device_printf(sc->dev, "device does not release the buffers; "
		    "they stay allocated\n");
		sc->keep_dma = 1;
		return;
	}
	sc->mem_window_set = 0;
}

/*
 * Sysctls.
 */
static int
ipts_hid_desc_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct ipts_softc *sc = arg1;

	if (sc->hid_desc == NULL)
		return (SYSCTL_OUT(req, "", 0));
	return (SYSCTL_OUT(req, sc->hid_desc, sc->hid_desc_len));
}

static int
ipts_last_frame_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct ipts_softc *sc = arg1;
	uint8_t buf[sizeof(sc->last_frame)];

	mtx_lock(&sc->mtx);
	memcpy(buf, sc->last_frame, sizeof(buf));
	mtx_unlock(&sc->mtx);
	return (SYSCTL_OUT(req, buf, sizeof(buf)));
}

static int
ipts_doorbell_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct ipts_softc *sc = arg1;
	uint32_t val;

	val = sc->doorbell.vaddr != NULL ? ipts_read_doorbell(sc) : 0;
	return (sysctl_handle_32(oidp, &val, 0, req));
}

/* The first 16 bytes of the header of each data buffer. */
static int
ipts_buffers_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct ipts_softc *sc = arg1;
	uint8_t buf[IPTS_BUFFERS * 16];
	int i;

	memset(buf, 0, sizeof(buf));
	for (i = 0; i < IPTS_BUFFERS; i++) {
		if (sc->data[i].vaddr == NULL)
			continue;
		bus_dmamap_sync(sc->data[i].tag, sc->data[i].map,
		    BUS_DMASYNC_POSTREAD);
		memcpy(&buf[i * 16], sc->data[i].vaddr, 16);
	}
	return (SYSCTL_OUT(req, buf, sizeof(buf)));
}

/* Input reports by report ID: "id:count" for each ID that came. */
static int
ipts_report_ids_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct ipts_softc *sc = arg1;
	struct sbuf sb;
	int error, i;

	sbuf_new_for_sysctl(&sb, NULL, 256, req);
	for (i = 0; i < 256; i++)
		if (sc->st_ids[i] != 0)
			sbuf_printf(&sb, "%s%#x:%lu", sbuf_len(&sb) > 0 ?
			    " " : "", i, sc->st_ids[i]);
	error = sbuf_finish(&sb);
	sbuf_delete(&sb);
	return (error);
}

static void
ipts_add_sysctls(struct ipts_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *list;

	ctx = device_get_sysctl_ctx(sc->dev);
	list = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	SYSCTL_ADD_U16(ctx, list, OID_AUTO, "vendor", CTLFLAG_RD,
	    &sc->vendor, 0, "Touch controller vendor");
	SYSCTL_ADD_U16(ctx, list, OID_AUTO, "product", CTLFLAG_RD,
	    &sc->product, 0, "Touch controller product");
	SYSCTL_ADD_INT(ctx, list, OID_AUTO, "mode", CTLFLAG_RD,
	    &sc->mode, 0, "0 = event, 1 = poll");
	SYSCTL_ADD_OPAQUE(ctx, list, OID_AUTO, "device_info", CTLFLAG_RD,
	    sc->info, sizeof(sc->info), "CU", "GET_DEVICE_INFO response");
	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "hid_descriptor",
	    CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    ipts_hid_desc_sysctl, "CU", "HID report descriptor");
	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "last_frame",
	    CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    ipts_last_frame_sysctl, "CU", "Start of the last data buffer");
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, "frames", CTLFLAG_RD,
	    &sc->st_frames, "Data buffers received");
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, "errors", CTLFLAG_RD,
	    &sc->st_errors, "Errors in the receive loop");
	SYSCTL_ADD_OPAQUE(ctx, list, OID_AUTO, "frame_types", CTLFLAG_RD,
	    sc->st_types, sizeof(sc->st_types), "LU",
	    "Data buffers for each type");
	SYSCTL_ADD_U32(ctx, list, OID_AUTO, "last_type", CTLFLAG_RD,
	    &sc->last_type, 0, "Type of the last data buffer");
	SYSCTL_ADD_U32(ctx, list, OID_AUTO, "last_size", CTLFLAG_RD,
	    &sc->last_size, 0, "Size of the last data buffer");
	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "doorbell",
	    CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    ipts_doorbell_sysctl, "IU", "Doorbell value");
	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "buffers",
	    CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    ipts_buffers_sysctl, "CU", "Header of each data buffer");
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, "ready", CTLFLAG_RD,
	    &sc->st_ready, "READY_FOR_DATA responses");
	SYSCTL_ADD_OPAQUE(ctx, list, OID_AUTO, "ready_rsp", CTLFLAG_RD,
	    sc->ready_rsp, sizeof(sc->ready_rsp), "CU",
	    "Payload of the last READY_FOR_DATA response");
	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "report_ids",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    ipts_report_ids_sysctl, "A", "Input reports by report ID");
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, "rsp_unexpected", CTLFLAG_RD,
	    &sc->st_rsp_unexpected, "Messages that are not responses");
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, "hid_reports", CTLFLAG_RD,
	    &sc->st_hid_reports, "Reports given to hidbus");
	SYSCTL_ADD_INT(ctx, list, OID_AUTO, "dump", CTLFLAG_RW,
	    &sc->dump, 0, "Print the next N data buffers");
}

/*
 * Device interface.
 */
static int
ipts_probe(device_t dev)
{
	if (!mei_uuid_match(mei_cl_props(dev)->uuid, IPTS_UUID))
		return (ENXIO);
	device_set_desc(dev, "Intel Precise Touch & Stylus");
	return (BUS_PROBE_DEFAULT);
}

static int
ipts_attach(device_t dev)
{
	struct ipts_softc *sc = device_get_softc(dev);
	int error;

	sc->dev = dev;
	sc->mode = ipts_mode == IPTS_MODE_POLL ? IPTS_MODE_POLL :
	    IPTS_MODE_EVENT;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	sx_init(&sc->cmd_lock, "ipts cmd");
	sx_init(&sc->feat_lock, "ipts feature");
	ipts_add_sysctls(sc);

	error = mei_cl_connect(dev);
	if (error != 0) {
		device_printf(dev, "cannot connect: %d\n", error);
		sx_destroy(&sc->feat_lock);
		sx_destroy(&sc->cmd_lock);
		mtx_destroy(&sc->mtx);
		return (error);
	}
	sc->connected = 1;

	error = ipts_start(sc);
	if (error != 0) {
		device_printf(dev, "start failed: %d\n", error);
		/* Stay attached, so that detach cleans up safely. */
		return (0);
	}
	device_printf(dev, "started in %s mode\n",
	    sc->mode == IPTS_MODE_POLL ? "poll" : "event");
	ipts_hid_attach(sc);
	return (0);
}

static int
ipts_detach(device_t dev)
{
	struct ipts_softc *sc = device_get_softc(dev);
	int error;

	error = bus_generic_detach(dev);
	if (error != 0)
		return (error);
	ipts_stop(sc);
	if (sc->connected)
		mei_cl_disconnect(dev);
	if (!sc->keep_dma)
		ipts_free_buffers(sc);
	free(sc->hid_desc, M_IPTS);
	free(sc->rdesc, M_IPTS);
	free(sc->feat_buf, M_IPTS);
	sx_destroy(&sc->feat_lock);
	sx_destroy(&sc->cmd_lock);
	mtx_destroy(&sc->mtx);
	return (0);
}

static device_method_t ipts_methods[] = {
	DEVMETHOD(device_probe,		ipts_probe),
	DEVMETHOD(device_attach,	ipts_attach),
	DEVMETHOD(device_detach,	ipts_detach),

	DEVMETHOD(hid_intr_setup,	ipts_hid_intr_setup),
	DEVMETHOD(hid_intr_unsetup,	ipts_hid_intr_unsetup),
	DEVMETHOD(hid_intr_start,	ipts_hid_intr_start),
	DEVMETHOD(hid_intr_stop,	ipts_hid_intr_stop),
	DEVMETHOD(hid_intr_poll,	ipts_hid_intr_poll),
	DEVMETHOD(hid_get_rdesc,	ipts_hid_get_rdesc),
	DEVMETHOD(hid_read,		ipts_hid_read),
	DEVMETHOD(hid_write,		ipts_hid_write),
	DEVMETHOD(hid_get_report,	ipts_hid_get_report),
	DEVMETHOD(hid_set_report,	ipts_hid_set_report),
	DEVMETHOD(hid_set_idle,		ipts_hid_set_idle),
	DEVMETHOD(hid_set_protocol,	ipts_hid_set_protocol),
	DEVMETHOD(hid_ioctl,		ipts_hid_ioctl),

	DEVMETHOD_END
};

static driver_t ipts_driver = {
	"ipts",
	ipts_methods,
	sizeof(struct ipts_softc),
};

extern driver_t hidbus_driver;

DRIVER_MODULE(ipts, mei, ipts_driver, 0, 0);
DRIVER_MODULE(hidbus, ipts, hidbus_driver, 0, 0);
MODULE_DEPEND(ipts, hid, 1, 1, 1);
MODULE_DEPEND(ipts, hidbus, 1, 1, 1);
