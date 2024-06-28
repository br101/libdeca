#include <blink.h>
#include <dwmac.h>
#include <dwproto.h>
#include <dwutil.h>
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
	tx->len = DWMAC_PROTO_SHORT_LEN + len;

	struct prot_short* ps = (struct prot_short*)tx->buf;
	/* prepare header */
	ps->hdr.fc = MAC154_FC_SHORT;
	ps->hdr.src = dwmac_get_mac16();
	ps->hdr.dst = dst;
	ps->hdr.panId = dwmac_get_panid();
	ps->hdr.seqNo = seqNo++;

	ps->func = func;

	return tx->buf + sizeof(struct prot_short);
}

/* len is user protocol length without headers */
void* dwprot_long_prepare(struct txbuf* tx, size_t len, uint8_t func,
						  uint64_t dst)
{
	dwmac_tx_prepare_null(tx);
	tx->len = DWMAC_PROTO_LONG_LEN + len;

	struct prot_long* pl = (struct prot_long*)tx->buf;
	/* prepare header */
	pl->hdr.fc = MAC154_FC_LONG;
	pl->hdr.dst = dst;
	pl->hdr.src = dwmac_get_mac64();

	pl->func = func;

	return tx->buf + sizeof(struct prot_long);
}

void* dwprot_prepare(struct txbuf* tx, size_t len, uint8_t func, uint64_t dst)
{
	if (IS_SHORT_ADDR(dst)) {
		return dwprot_short_prepare(tx, len, func, (uint16_t)dst);
	} else {
		return dwprot_long_prepare(tx, len, func, dst);
	}
}

/* len is user protocol length without headers */
void* dwprot_long_src_prepare(struct txbuf* tx, size_t len, uint8_t func,
							  uint64_t src)
{
	dwmac_tx_prepare_null(tx);
	tx->len = DWMAC_PROTO_LONG_SRC_LEN + len;

	struct prot_long_src* pl = (struct prot_long_src*)tx->buf;
	/* prepare header */
	pl->hdr.fc = MAC154_FC_LONG_SRC;
	pl->hdr.src = src;

	pl->func = func;

	return tx->buf + sizeof(struct prot_long_src);
}

uint64_t dwprot_get_src(const uint8_t* buf)
{
	const struct prot_short* ps = (const struct prot_short*)buf;
	if (ps->hdr.fc == MAC154_FC_SHORT) {
		return ps->hdr.src;
	} else if (ps->hdr.fc == MAC154_FC_LONG) {
		const struct prot_long* pl = (const struct prot_long*)buf;
		return pl->hdr.src;
	} else if (ps->hdr.fc == MAC154_FC_LONG_SRC) {
		const struct prot_long_src* pl = (const struct prot_long_src*)buf;
		return pl->hdr.src;
	}
	return 0;
}

uint8_t dwprot_get_func(const uint8_t* buf)
{
	const struct prot_short* ps = (const struct prot_short*)buf;
	if (ps->hdr.fc == MAC154_FC_SHORT) {
		return ps->func;
	} else if (ps->hdr.fc == MAC154_FC_LONG) {
		const struct prot_long* pl = (const struct prot_long*)buf;
		return pl->func;
	} else if (ps->hdr.fc == MAC154_FC_LONG_SRC) {
		const struct prot_long_src* pl = (const struct prot_long_src*)buf;
		return pl->func;
	}
	return 0;
}

size_t dwprot_get_payload_len(const uint8_t* buf, size_t len)
{
	uint16_t fc = *(uint16_t*)buf;
	if (fc == MAC154_FC_SHORT) {
		return len - DWMAC_PROTO_SHORT_LEN;
	} else if (fc == MAC154_FC_LONG) {
		return len - DWMAC_PROTO_LONG_LEN;
	} else if (fc == MAC154_FC_LONG_SRC) {
		return len - DWMAC_PROTO_LONG_SRC_LEN;
	}
	return 0;
}

const void* dwprot_get_payload(const uint8_t* buf)
{
	const struct prot_short* ps = (const struct prot_short*)buf;
	if (ps->hdr.fc == MAC154_FC_SHORT) {
		return ps->pbuf;
	} else if (ps->hdr.fc == MAC154_FC_LONG) {
		const struct prot_long* pl = (const struct prot_long*)buf;
		return pl->pbuf;
	} else if (ps->hdr.fc == MAC154_FC_LONG_SRC) {
		const struct prot_long_src* pl = (const struct prot_long_src*)buf;
		return pl->pbuf;
	}
	return NULL;
}

bool dwprot_check_min_len(const uint8_t* buf, size_t len)
{
	if (len < 2) {
		return false; // not even FC
	}

	uint16_t fc = *(uint16_t*)buf;
	if (fc == MAC154_FC_SHORT) {
		return len >= DWMAC_PROTO_SHORT_LEN;
	} else if (fc == MAC154_FC_LONG) {
		return len >= DWMAC_PROTO_LONG_LEN;
	} else if (fc == MAC154_FC_LONG_SRC) {
		return len >= DWMAC_PROTO_LONG_SRC_LEN;
	}
	return false;
}

void dwprot_rx_handler(const struct rxbuf* rx)
{
	if (rx->len < 2) {
		return; // too short
	}

	uint16_t fc = *(uint16_t*)rx->buf;
	// LOG_INF("RX len %d FC 0x%04X", rx->len, fc);

	// Note: handle 1 byte FC for blink first!
	if ((uint8_t)fc == MAC154_FC_BLINK_SHORT) {
		blink_handle_msg_short(rx);
	} else if ((uint8_t)fc == MAC154_FC_BLINK_LONG) {
		blink_handle_msg_long(rx);
	} else if (fc & MAC154_FC_TYPE_DATA) {
		if (!dwprot_check_min_len(rx->buf, rx->len)) {
			// checks also frame types we are responsible for
			return;
		}
		uint8_t func = dwprot_get_func(rx->buf);
		if ((func & DWMAC_PROTO_MSG_MASK) == TWR_MSG_GROUP) {
			twr_handle_message(rx);
		} else if (func == SYNC_MSG) {
			sync_handle_msg(rx);
		}
	}
}
