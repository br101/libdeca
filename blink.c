#include "blink.h"
#include "dwmac.h"
#include "dwphy.h"
#include "dwproto.h"
#include "dwtime.h"
#include "dwutil.h"
#include "log.h"

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
	struct prot_short* ps = (struct prot_short*)rx->buf;
	const struct blink_msg* msg = (struct blink_msg*)ps->pbuf;

	LOG_DBG("BLINK #%lu " ADDR_FMT " " DWT_FMT " (%x)", msg->seq_no,
			ps->hdr.src, DWT_PAR(rx->ts), msg->battery);

	uint64_t rx_ts = dw_timestamp_extend(rx->ts);

	if (blink_cb) {
		blink_cb(ps->hdr.src, msg->seq_no, rx_ts, msg->time_ms, msg->battery);
	}
}

bool blink_send_short(void)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL)
		return false;

	struct blink_msg* msg
		= dwprot_short_prepare(tx, sizeof(struct blink_msg), BLINK_MSG, 0xffff);
	msg->seq_no = blink_seq++;
	msg->time_ms = 0; // TODO plat_get_time();
	msg->battery = 0; // TODO plat_get_battery();

	bool res = dwmac_tx_queue(tx);
	LOG_TX_RES(res, "BLINK #%lu", msg->seq_no);
	return res;
}

void blink_handle_msg_long(const struct rxbuf* rx)
{
	const struct mac154_hdr_blink_long* bh
		= (const struct mac154_hdr_blink_long*)rx->buf;
	const struct blink_msg* msg
		= (struct blink_msg*)(rx->buf + sizeof(struct mac154_hdr_blink_long));

	if (rx->len < BLINK_LONG_SIZE) {
		return;
	}

	LOG_DBG("BLINK LONG #%lu " LADDR_FMT " " DWT_FMT " (%x)", msg->seq_no,
			bh->src, DWT_PAR(rx->ts), msg->battery);

	uint64_t rx_ts = dw_timestamp_extend(rx->ts);

	if (blink_cb) {
		blink_cb(bh->src, msg->seq_no, rx_ts, msg->time_ms, msg->battery);
	}
}

bool blink_send_long(uint64_t src)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL)
		return false;

	dwmac_tx_prepare(tx, BLINK_LONG_SIZE);

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
	LOG_TX_RES(res, "BLINK LONG #%lu", msg->seq_no);
	return res;
}

void blink_set_handler(blink_cb_t cb)
{
	blink_cb = cb;
}
