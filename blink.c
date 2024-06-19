#include "blink.h"
#include "dwmac.h"
#include "dwphy.h"
#include "dwtime.h"
#include "dwutil.h"
#include "log.h"
#include "sync.h"

struct blink_msg {
	uint32_t seq_no;
	uint64_t time_ms; // system time in ms
	uint8_t battery;
} __attribute__((packed));

static const char* LOG_TAG = "BLINK";
static blink_cb_t blink_cb;
static uint32_t blink_seq;

void blink_handle_msg(const struct rxbuf* rx)
{
	const struct blink_msg* msg = (struct blink_msg*)rx->u.s.pbuf;

	LOG_DBG("BLINK #%lu " ADDR_FMT " " DWT_FMT " (%x)", msg->seq_no,
			rx->u.s.hdr.src, DWT_PAR(rx->ts), msg->battery);

	uint64_t rx_ts = dw_timestamp_extend(rx->ts);

	if (blink_cb) {
		blink_cb(rx->u.s.hdr.src, msg->seq_no, rx_ts, msg->time_ms,
				 msg->battery);
	}
}

bool blink_send(void)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL)
		return false;

	struct blink_msg* msg = (struct blink_msg*)tx->u.s.pbuf;
	msg->seq_no = blink_seq++;
	msg->time_ms = 0; // TODO plat_get_time();
	msg->battery = 0; // TODO plat_get_battery();

	dwmac_tx_prepare_prot(tx, sizeof(struct blink_msg), BLINK_MSG, 0xffff);

	bool res = dwmac_tx_queue(tx);
	LOG_TX_RES(res, "BLINK #%d", tx->u.s.hdr.seqNo);
	return res;
}

void blink_set_handler(blink_cb_t cb)
{
	blink_cb = cb;
}
