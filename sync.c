#include "sync.h"
#include "dwmac.h"
#include "dwphy.h"
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

bool sync_send(void)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL)
		return false;

	/* next possible sending time with some slack */
	uint64_t send_dtu = dw_get_systime() + MS_TO_DTU(10);
	send_dtu &= DTU_MASK;

	struct toda_sync_msg* msg = (struct toda_sync_msg*)tx->u.s.pbuf;
	msg->tx_ts = send_dtu + DWPHY_ANTENNA_DELAY;
	msg->seq_no = sync_seq++;

	dwmac_tx_prepare_prot(tx, sizeof(struct toda_sync_msg), SYNC_MSG, 0xffff);
	dwmac_tx_set_txtime(tx, send_dtu);

	bool res = dwmac_tx_queue(tx);
	LOG_TX_RES(res, "Sync #%d", tx->u.s.hdr.seqNo);
	return res;
}

void sync_handle_msg(const struct rxbuf* rx)
{
	LOG_DBG("Received SYNC #%d from " ADDR_FMT, rx->u.s.hdr.seqNo,
			rx->u.s.hdr.src);
	const struct toda_sync_msg* msg = (struct toda_sync_msg*)rx->u.s.pbuf;
	// LOG_DBGL_TS(DDL_TDOA, "\tTX TS*: ", msg->tx_ts);
	// LOG_DBGL_TS(DDL_TDOA, "\tRX TS: ", rx->ts);

#if DWMAC_USE_CARRIERINTEG
	float skewci = dwphy_get_rx_clock_offset_ci(rx->ci) * -1.0;
#else
	float skewci = 0.0;
#endif

#if DEBUG && NO_FLOAT_PRINTF
	LOG_DBG("SYNC #%d " ADDR_FMT " " DWT_FMT " " DWT_FMT " %s %s",
			mb->s.hdr.seqNo, mb->s.hdr.src, DWT_PAR(tx_ts), DWT_PAR(rx->ts),
			double_to_sstr(skewci));
#else
	LOG_DBG("SYNC #%lu " ADDR_FMT " " DWT_FMT " " DWT_FMT " %.2f", msg->seq_no,
			rx->u.s.hdr.src, DWT_PAR(msg->tx_ts), DWT_PAR(rx->ts), skewci);
#endif

	if (sync_cb) {
		sync_cb(rx->u.s.hdr.src, msg->seq_no, msg->tx_ts, rx->ts, skewci);
	}
}

void sync_set_handler(sync_cb_t cb)
{
	sync_cb = cb;
}
