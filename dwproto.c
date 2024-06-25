#include <dwmac.h>
#include <dwproto.h>
#include <mac802154.h>

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
