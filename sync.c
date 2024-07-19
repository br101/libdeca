/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#include "sync.h"
#include "dwmac.h"
#include "dwphy.h"
#include "dwproto.h"
#include "dwtime.h"
#include "dwutil.h"
#include "log.h"

struct toda_sync_msg {
	uint32_t seq_no;
	uint64_t tx_ts;
} __attribute__((packed));

static const char* LOG_TAG = "SYNC";
static sync_cb_t sync_cb;
static uint32_t sync_seq;

bool sync_send_short(void)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL)
		return false;

	/* next possible sending time with some slack */
	uint64_t send_dtu = dw_get_systime();
	send_dtu = dw_timestamp_extend(send_dtu);
	send_dtu += MS_TO_DTU(10);

	struct toda_sync_msg* msg = dwprot_short_prepare(
		tx, sizeof(struct toda_sync_msg), SYNC_MSG, 0xffff);
	msg->tx_ts = send_dtu + DWPHY_ANTENNA_DELAY;
	msg->seq_no = sync_seq++;

	send_dtu &= DTU_MASK;
	dwmac_tx_set_txtime(tx, send_dtu);

	bool res = dwmac_transmit(tx);
	LOG_TX_RES(res, "Sync #%lu", msg->seq_no);
	return res;
}

bool sync_send_long(uint64_t src)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL)
		return false;

	/* next possible sending time with some slack */
	uint64_t send_dtu = dw_get_systime();
	send_dtu = dw_timestamp_extend(send_dtu);
	send_dtu += MS_TO_DTU(10);

	struct toda_sync_msg* msg = dwprot_long_src_prepare(
		tx, sizeof(struct toda_sync_msg), SYNC_MSG, src);
	msg->tx_ts = send_dtu + DWPHY_ANTENNA_DELAY;
	msg->seq_no = sync_seq++;

	send_dtu &= DTU_MASK;
	dwmac_tx_set_txtime(tx, send_dtu);

	bool res = dwmac_transmit(tx);
	LOG_TX_RES(res, "Sync long #%lu", msg->seq_no);
	return res;
}

void sync_handle_msg(const struct rxbuf* rx)
{
	if (dwprot_get_payload_len(rx->buf, rx->len)
		!= sizeof(struct toda_sync_msg)) {
		LOG_ERR("Invalid sized message");
		return;
	}

	uint64_t src = dwprot_get_src(rx->buf);
	const struct toda_sync_msg* msg = dwprot_get_payload(rx->buf);

	// LOG_DBGL_TS(DDL_TDOA, "\tTX TS*: ", msg->tx_ts);
	// LOG_DBGL_TS(DDL_TDOA, "\tRX TS: ", rx->ts);

	LOG_DBG("Received SYNC %lu from " LADDR_FMT, msg->seq_no, LADDR_PAR(src));

#if CONFIG_DECA_USE_CARRIERINTEG
	float skew = dwphy_get_rx_clock_offset_ci(rx->ci) * -1.0;
#else
	float skew = (float)dwt_readclockoffset() / (uint32_t)(1 << 26) * 1.0e6;
#endif

#if DEBUG && NO_FLOAT_PRINTF
	LOG_DBG("SYNC #%d " ADDR_FMT " " DWT_FMT " " DWT_FMT " %s %s",
			mb->s.hdr.seqNo, mb->s.hdr.src, DWT_PAR(tx_ts), DWT_PAR(rx->ts),
			double_to_sstr(skew));
#else
	LOG_DBG("SYNC LONG #%lu " LADDR_FMT " " DWT_FMT " " DWT_FMT " %f",
			msg->seq_no, LADDR_PAR(src), DWT_PAR(msg->tx_ts), DWT_PAR(rx->ts),
			skew);
#endif

	uint64_t rx_ts = dw_timestamp_extend(rx->ts);

	if (sync_cb) {
		sync_cb(src, msg->seq_no, msg->tx_ts, rx_ts, skew);
	}
}

void sync_set_observer(sync_cb_t cb)
{
	sync_cb = cb;
}
