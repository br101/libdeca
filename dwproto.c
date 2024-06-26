#include <blink.h>
#include <dwmac.h>
#include <dwproto.h>
#include <mac802154.h>
#include <ranging.h>
#include <sync.h>

#include "log.h"

// static const char* LOG_TAG = "PROTO";
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

/* len is user protocol length without headers */
void* dwprot_long_src_prepare(struct txbuf* tx, size_t len, uint8_t func,
							  uint64_t src)
{
	dwmac_tx_prepare_null(tx);
	tx->len = DWMAC_PROTO_LONG_LEN + len;

	struct prot_long_src* pl = (struct prot_long_src*)tx->buf;
	/* prepare header */
	pl->hdr.fc = MAC154_FC_TYPE_DATA | MAC154_FC_LONG_SRC;
	pl->hdr.src = src;

	pl->func = func;

	return tx->buf + sizeof(struct prot_long_src);
}

void dwprot_rx_handler(const struct rxbuf* rx)
{
	uint16_t fc = *(uint16_t*)rx->buf;
	// LOG_INF("RX len %d FC 0x%04X", rx->len, fc);

	if (rx->len < DWMAC_PROTO_MIN_LEN) {
		return; // too short
	}

	// Note: handle 1 byte FC for blink first!
	if ((uint8_t)fc == MAC154_FC_BLINK_SHORT) {
		blink_handle_msg_short(rx);
	} else if ((uint8_t)fc == MAC154_FC_BLINK_LONG) {
		blink_handle_msg_long(rx);
	} else if (fc == (MAC154_FC_TYPE_DATA | MAC154_FC_SHORT)) {
		const struct prot_short* ps = (const struct prot_short*)rx->buf;
		if ((ps->func & DWMAC_PROTO_MSG_MASK) == TWR_MSG_GROUP) {
			twr_handle_message_short(rx);
		} else if (ps->func == SYNC_MSG) {
			sync_handle_msg_short(rx);
		}
	} else if (fc == (MAC154_FC_TYPE_DATA | MAC154_FC_LONG_SRC)) {
		const struct prot_long_src* pl = (const struct prot_long_src*)rx->buf;
		if (pl->func == SYNC_MSG) {
			sync_handle_msg_long(rx);
		}
	}
}
