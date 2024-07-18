/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#include "blink.h"
#include "dwmac.h"
#include "dwphy.h"
#include "dwproto.h"
#include "dwtime.h"
#include "dwutil.h"
#include "log.h"

#define BLINK_SHORT_SIZE                                                       \
	(sizeof(struct mac154_hdr_blink_short) + sizeof(struct blink_msg))
#define BLINK_LONG_SIZE                                                        \
	(sizeof(struct mac154_hdr_blink_long) + sizeof(struct blink_msg))

struct blink_msg {
	uint32_t seq_no;
	uint64_t time_ms; // system time in ms
	uint8_t battery;
} __attribute__((packed));

static const char* LOG_TAG = "BLINK";
static blink_cb_t blink_cb;
static uint32_t blink_seq;

void blink_handle_msg_short(const struct rxbuf* rx)
{
	if (rx->len < BLINK_SHORT_SIZE) {
		return;
	}

	const struct mac154_hdr_blink_short* bh
		= (const struct mac154_hdr_blink_short*)rx->buf;
	const struct blink_msg* msg
		= (struct blink_msg*)(rx->buf + sizeof(struct mac154_hdr_blink_short));

	LOG_DBG("BLINK #%lu " ADDR_FMT " " DWT_FMT " (%x)", msg->seq_no, bh->src,
			DWT_PAR(rx->ts), msg->battery);

	uint64_t rx_ts = dw_timestamp_extend(rx->ts);

	if (blink_cb) {
		blink_cb(bh->src, msg->seq_no, rx_ts, msg->time_ms, msg->battery);
	}
}

bool blink_send_short(uint16_t src)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL) {
		return false;
	}

	dwmac_tx_prepare(tx, BLINK_SHORT_SIZE);

	struct mac154_hdr_blink_short* bh = (struct mac154_hdr_blink_short*)tx->buf;
	bh->fc = MAC154_FC_BLINK_SHORT;
	bh->seqNo = blink_seq++; // will be truncated
	bh->src = src;

	struct blink_msg* msg
		= (struct blink_msg*)(tx->buf + sizeof(struct mac154_hdr_blink_short));
	msg->seq_no = blink_seq;
	msg->time_ms = 0; // TODO plat_get_time();
	msg->battery = 0; // TODO plat_get_battery();

	bool res = dwmac_tx_queue(tx);
	LOG_TX_RES(res, "BLINK #%lu " ADDR_FMT, msg->seq_no, src);
	return res;
}

void blink_handle_msg_long(const struct rxbuf* rx)
{
	if (rx->len < BLINK_LONG_SIZE) {
		return;
	}

	const struct mac154_hdr_blink_long* bh
		= (const struct mac154_hdr_blink_long*)rx->buf;
	const struct blink_msg* msg
		= (struct blink_msg*)(rx->buf + sizeof(struct mac154_hdr_blink_long));

	LOG_DBG("BLINK #%lu " LADDR_FMT " " DWT_FMT " (%x)", msg->seq_no,
			LADDR_PAR(bh->src), DWT_PAR(rx->ts), msg->battery);

	uint64_t rx_ts = dw_timestamp_extend(rx->ts);

	if (blink_cb) {
		blink_cb(bh->src, msg->seq_no, rx_ts, msg->time_ms, msg->battery);
	}
}

bool blink_send_long(uint64_t src, bool sleep_after_tx)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL) {
		return false;
	}

	dwmac_tx_prepare(tx, BLINK_LONG_SIZE);
	if (sleep_after_tx) {
		dwmac_tx_set_sleep_after_tx(tx);
	}

	struct mac154_hdr_blink_long* bh = (struct mac154_hdr_blink_long*)tx->buf;
	bh->fc = MAC154_FC_BLINK_LONG;
	bh->seqNo = blink_seq++; // will be truncated
	bh->src = src;

	struct blink_msg* msg
		= (struct blink_msg*)(tx->buf + sizeof(struct mac154_hdr_blink_long));
	msg->seq_no = blink_seq;
	msg->time_ms = 0; // TODO plat_get_time();
	msg->battery = 0; // TODO plat_get_battery();

	bool res = dwmac_tx_queue(tx);
	LOG_TX_RES(res, "BLINK #%lu " LADDR_FMT, msg->seq_no, LADDR_PAR(src));
	return res;
}

void blink_set_observer(blink_cb_t cb)
{
	blink_cb = cb;
}
