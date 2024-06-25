#include <blink.h>
#include <dwmac.h>
#include <dwproto.h>
#include <mac802154.h>
#include <ranging.h>
#include <sync.h>

#include "log.h"

static const char* LOG_TAG = "PROTO";
static uint8_t seqNo;

/* len is user protocol length without headers */
void* dwprot_short_prepare(struct txbuf* tx, size_t len, uint8_t func,
						   uint16_t dst)
{
	dwmac_tx_prepare_null(tx);
	tx->len = DWMAC_PROTO_MIN_LEN + len;

	struct prot_short* ps = (struct prot_short*)tx->buf;
	/* prepare header */
	ps->hdr.fc = MAC154_FC_TYPE_DATA | MAC154_FC_SHORT;
	ps->hdr.src = dwmac_get_mac16();
	ps->hdr.dst = dst;
	ps->hdr.panId = dwmac_get_panid();
	ps->hdr.seqNo = seqNo++;

	ps->func = func;

	return tx->buf + sizeof(struct prot_short);
}

void dwprot_rx_handler(const struct rxbuf* rx)
{
	// LOG_INF("RX %d fc %x", rx->len, rx->buf[0]);

	if (rx->len < DWMAC_PROTO_MIN_LEN) {
		return; // too short
	}

	const struct prot_short* ps = (const struct prot_short*)rx->buf;
	const struct mac154_hdr_blink_long* bh
		= (const struct mac154_hdr_blink_long*)rx->buf;

	if (ps->hdr.fc == (MAC154_FC_TYPE_DATA | MAC154_FC_SHORT)) {
		if ((ps->func & DWMAC_PROTO_MSG_MASK) == TWR_MSG_GROUP) {
			twr_handle_message_short(rx);
		} else if (ps->func == BLINK_MSG) {
			blink_handle_msg_short(rx);
		} else if (ps->func == SYNC_MSG) {
			sync_handle_msg_short(rx);
		}
	} else if (bh->fc == MAC154_FC_BLINK_LONG) {
		blink_handle_msg_long(rx);
	}
}
