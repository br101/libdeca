#ifndef MAC_802_15_4
#define MAC_802_15_4

#include <stdint.h>

#define MAC154_FC_TYPE_MASK		 0x0007 /* Frame Type */
#define MAC154_FC_SECURITY		 0x0008 /* Security Enabled */
#define MAC154_FC_FRAME_PEND	 0x0010 /* Frame Pending */
#define MAC154_FC_ACK_REQ		 0x0020 /* AR ack required */
#define MAC154_FC_PAN_ID_COMP	 0x0040 /* PAN ID Compression */
#define MAC154_FC_SEQ_SUPP		 0x0100 /* Sequence Number Suppression */
#define MAC154_FC_IE_PRESENT	 0x0200 /* IE Present */
#define MAC154_FC_DST_ADDR_MASK	 0x0C00 /* Destination Addressing Mode */
#define MAC154_FC_DST_ADDR_NONE	 0x0000 /* PAN ID and address not present */
#define MAC154_FC_DST_ADDR_SHORT 0x0800 /* 16bit address */
#define MAC154_FC_DST_ADDR_LONG	 0x0C00 /* 64bit address */
#define MAC154_FC_VERSION_MASK	 0x3000
#define MAC154_FC_VERSION_0		 0x0000 /* 820.15.4-2003 */
#define MAC154_FC_VERSION_1		 0x1000 /* 820.15.4-2006 */
#define MAC154_FC_VERSION_2		 0x2000 /* 820.15.4-2020 */
#define MAC154_FC_VERSION_3		 0x3000 /* reserved */
#define MAC154_FC_SRC_ADDR_MASK	 0xC000
#define MAC154_FC_SRC_ADDR_NONE	 0x0000
#define MAC154_FC_SRC_ADDR_SHORT 0x8000
#define MAC154_FC_SRC_ADDR_LONG	 0xC000

/* frame types */
#define MAC154_FC_TYPE_BEACON	0x0
#define MAC154_FC_TYPE_DATA		0x1
#define MAC154_FC_TYPE_ACK		0x2
#define MAC154_FC_TYPE_COMMAND	0x3
#define MAC154_FC_TYPE_RES		0x4 /* reserved */
#define MAC154_FC_TYPE_MULTI	0x5
#define MAC154_FC_TYPE_FRAG		0x6
#define MAC154_FC_TYPE_EXTENDED 0x7

#define MAC154_SEC_LVL_NONE			  0x00
#define MAC154_SEC_LVL_MIC32		  0x01
#define MAC154_SEC_LVL_MIC64		  0x02
#define MAC154_SEC_LVL_MIC128		  0x03
#define MAC154_SEC_LVL_RES			  0x04 /* reserved */
#define MAC154_SEC_LVL_ENCMIC32		  0x05
#define MAC154_SEC_LVL_ENCMIC64		  0x06
#define MAC154_SEC_LVL_ENCMIC128	  0x07
#define MAC154_SEC_KEY_ID_IMPLICIT	  0x00
#define MAC154_SEC_KEY_ID_INDEX		  0x08
#define MAC154_SEC_KEY_ID_4BYTE		  0x10
#define MAC154_SEC_KEY_ID_8BYTE		  0x18
#define MAC154_SEC_FRAME_COUNTER_SUPP 0x20
#define MAC154_SEC_ASN_IN_NONCE		  0x40

#define MAC154_IE_TYPE_HEADER	 0x0
#define MAC154_IE_TYPE_PAYLOAD	 0x1
#define MAC154_HEADER_IE_VENDOR	 0x00
#define MAC154_HEADER_IE_TERM1	 0x7e
#define MAC154_HEADER_IE_TERM2	 0x7f
#define MAC154_PAYLOAD_IE_VENDOR 0x2
#define MAC154_PAYLOAD_IE_TERM	 0xf

/* multipurpose frame short FC fields */
#define MAC154_FC_MULTI_TYPE_MASK	   0x07
#define MAC154_FC_MULTI_LONG_FC		   0x08
#define MAC154_FC_MULTI_DST_ADDR_MASK  0x30
#define MAC154_FC_MULTI_DST_ADDR_NONE  0x00
#define MAC154_FC_MULTI_DST_ADDR_SHORT 0x10
#define MAC154_FC_MULTI_DST_ADDR_LONG  0x30
#define MAC154_FC_MULTI_SRC_ADDR_MASK  0xC0
#define MAC154_FC_MULTI_SRC_ADDR_NONE  0x00
#define MAC154_FC_MULTI_SRC_ADDR_SHORT 0x40
#define MAC154_FC_MULTI_SRC_ADDR_LONG  0xC0

/* multipurpose frame long FC fields */
#define MAC154_FC_MULTI_PANID		 0x0100
#define MAC154_FC_MULTI_SECURITY	 0x0200
#define MAC154_FC_MULTI_SEQNUM_SUPP	 0x0400
#define MAC154_FC_MULTI_FRAME_PEND	 0x0800
#define MAC154_FC_MULTI_VERSION_MASK 0x3000
#define MAC154_FC_MULTI_ACK_REQ		 0x4000
#define MAC154_FC_MULTI_IE_PRESENT	 0x4000

/* "normal" short frame */
#define MAC154_FC_SHORT                                                        \
	(MAC154_FC_DST_ADDR_SHORT | MAC154_FC_SRC_ADDR_SHORT                       \
	 | MAC154_FC_PAN_ID_COMP)

/* long frame with only source address */
#define MAC154_FC_LONG_SRC                                                     \
	(MAC154_FC_VERSION_2 | MAC154_FC_SRC_ADDR_LONG | MAC154_FC_SEQ_SUPP        \
	 | MAC154_FC_PAN_ID_COMP)

#define MAC154_FC_BLINK_SHORT                                                  \
	(MAC154_FC_TYPE_MULTI | MAC154_FC_MULTI_SRC_ADDR_SHORT)

#define MAC154_FC_BLINK_LONG                                                   \
	(MAC154_FC_TYPE_MULTI | MAC154_FC_MULTI_SRC_ADDR_LONG)

#define MAC154_FCS_LEN 2

#define MAC154_IE 0

struct mac154_hdr_ie {
	uint16_t len : 7;
	uint16_t id : 8;
	uint16_t type : 1;
} __attribute__((packed));

struct mac154_pld_ie {
	uint16_t len : 11;
	uint16_t id : 4;
	uint16_t type : 1;
} __attribute__((packed));

struct mac154_hdr_short {
	uint16_t fc;
	uint8_t seqNo;
	uint16_t panId;
	uint16_t dst;
	uint16_t src;
} __attribute__((packed));

struct mac154_hdr_dst_sec {
	uint16_t fc;
	uint16_t dst;
	uint8_t sec;
} __attribute__((packed));

struct mac154_ack {
	uint16_t fc;
	uint8_t seqNo;
	uint16_t fcs;
} __attribute__((packed));

struct mac154_hdr_blink_short {
	uint8_t fc;
	uint8_t seqNo;
	uint16_t src;
} __attribute__((packed));

struct mac154_hdr_blink_long {
	uint8_t fc;
	uint8_t seqNo;
	uint64_t src;
} __attribute__((packed));

struct mac154_hdr_long_src {
	uint16_t fc;
	uint64_t src;
} __attribute__((packed));

void mac154_set_frame_pending(uint8_t* buf);

#endif
