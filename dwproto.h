#ifndef DECA_PROTOCOL_H
#define DECA_PROTOCOL_H

#include "dwmac.h"
#include "mac802154.h"

#define DWMAC_PROTO_MSG_MASK	 0xF0
#define DWMAC_PROTO_SHORT_LEN	 (sizeof(struct prot_short) + MAC154_FCS_LEN)
#define DWMAC_PROTO_LONG_LEN	 (sizeof(struct prot_long) + MAC154_FCS_LEN)
#define DWMAC_PROTO_LONG_SRC_LEN (sizeof(struct prot_long_src) + MAC154_FCS_LEN)

/* for convienient access to mac154 header and buffer and "our protocol"
 * function or packet type */

struct prot_short {
	struct mac154_hdr_short hdr;
	uint8_t func; // frame type
	uint8_t pbuf[0];
} __attribute__((packed));

struct prot_long {
	struct mac154_hdr_long hdr;
	uint8_t func; // frame type
	uint8_t pbuf[0];
} __attribute__((packed));

struct prot_long_src {
	struct mac154_hdr_long_src hdr;
	uint8_t func; // frame type
	uint8_t pbuf[0];
} __attribute__((packed));

/* return pointer to space after header and func */
void* dwprot_short_prepare(struct txbuf* tx, size_t len, uint8_t func,
						   uint16_t dst);
void* dwprot_long_prepare(struct txbuf* tx, size_t len, uint8_t func,
						  uint64_t dst);
/** prepare either short or long */
void* dwprot_prepare(struct txbuf* tx, size_t len, uint8_t func, uint64_t dst);
void* dwprot_long_src_prepare(struct txbuf* tx, size_t len, uint8_t func,
							  uint64_t src);

void dwprot_rx_handler(const struct rxbuf* rx);

/** get from either short or long */
uint64_t dwprot_get_src(const uint8_t* buf);
uint8_t dwprot_get_func(const uint8_t* buf);
const void* dwprot_get_payload(const uint8_t* buf);
bool dwprot_check_min_len(const uint8_t* buf, size_t len);
size_t dwprot_get_payload_len(const uint8_t* buf, size_t len);

#endif
