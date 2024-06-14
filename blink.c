#include "blink.h"
#include "dwmac.h"
#include "dwphy.h"
#include "dwtime.h"
#include "dwutil.h"
#include "log.h"
#include "sync.h"

struct blink_msg {
	uint8_t battery; // not part of TDOA
} __attribute__((packed));

static const char* LOG_TAG = "BLINK";

void blink_handle_msg(const struct rxbuf* rx)
{
	const struct blink_msg* msg = (struct blink_msg*)rx->u.s.pbuf;

	LOG_INF("BLINK #%d " ADDR_FMT " " DWT_FMT " (%x)", rx->u.s.hdr.seqNo,
			rx->u.s.hdr.src, DWT_PAR(rx->ts), msg->battery);
}

bool blink_send(void)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL)
		return false;

	// struct tdoa_blink_msg* msg = (struct tdoa_blink_msg*)tx->u.s.pbuf;
	// msg->battery = tag_get_battery();

	dwmac_tx_prepare_prot(tx, sizeof(struct blink_msg), BLINK_MSG, 0xffff);

	bool res = dwmac_tx_queue(tx);
	LOG_TX_RES(res, "BLINK #%d", tx->u.s.hdr.seqNo);
	return res;
}
