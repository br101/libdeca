#ifndef DECA_PROTOCOL_H
#define DECA_PROTOCOL_H

#include "dwmac.h"
#include "mac802154.h"

#define DWMAC_PROTO_MSG_MASK 0xF0
#define DWMAC_PROTO_MIN_LEN	 (sizeof(struct prot_short) + MAC154_FCS_LEN)
#define DWMAC_PROTO_LONG_LEN (sizeof(struct prot_long_src) + MAC154_FCS_LEN)
#define DWMAC_PROTO_MAX_LEN	 (DWMAC_RXBUF_LEN - DWMAC_PROTO_MIN_LEN)

/* for convienient access to mac154 header and buffer and "our protocol"
 * function or packet type */
struct prot_short {
	struct mac154_hdr_short hdr;
	uint8_t func; // frame type
	uint8_t pbuf[0];
} __attribute__((packed));

/* for convienient access to mac154 header and buffer and "our protocol"
 * function or packet type */
struct prot_long_src {
	struct mac154_hdr_long_src hdr;
	uint8_t func; // frame type
	uint8_t pbuf[0];
} __attribute__((packed));

/*** short mac addresses standard header ***/

/* return pointer to space after header and func */
void* dwprot_short_prepare(struct txbuf* tx, size_t len, uint8_t func,
						   uint16_t dst);
void* dwprot_long_src_prepare(struct txbuf* tx, size_t len, uint8_t func,
							  uint64_t src);

void dwprot_rx_handler(const struct rxbuf* rx);

#endif
