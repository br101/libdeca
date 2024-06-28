#ifndef DECA_MAC_H
#define DECA_MAC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <deca_device_api.h>

#define CONFIG_DECA_DEBUG_IRQ_TIME	   0
#define CONFIG_DECA_DEBUG_RX_DUMP	   0
#define CONFIG_DECA_DEBUG_RX_STATUS	   1
#define CONFIG_DECA_DEBUG_TX_DUMP	   0
#define CONFIG_DECA_DEBUG_TX_TIME	   0
#define CONFIG_DECA_DEBUG_FRAME_FILTER 1

#define DWMAC_USE_CARRIERINTEG 1
#define DWMAC_INCLUDE_RXDIAG   0
#define DWMAC_XTAL_TRIM		   0

/* Buffer length is optimized for FIRA at the moment */
#define DWMAC_RXBUF_LEN 70

#define LOG_TX_RES(_res, ...)                                                  \
	do {                                                                       \
		if (res)                                                               \
			LOG_INF("Sent " __VA_ARGS__);                                      \
		else                                                                   \
			LOG_ERR("Failed to queue " __VA_ARGS__);                           \
	} while (0)

struct rxbuf {
	uint8_t buf[DWMAC_RXBUF_LEN];
	size_t len;
	uint64_t ts; /* RX timestamp from DW3000 */
#if CONFIG_DECA_DEBUG_IRQ_TIME
	uint64_t ts_irq_start; /* IRQ start timestamp */
	uint64_t ts_irq_end;   /* IRQ end timestamp */
#endif
#if DWMAC_USE_CARRIERINTEG
	int32_t ci; /* carrier integrator for clock offset */
#endif
#if DWMAC_INCLUDE_RXDIAG
	dwt_rxdiag_t diag;
#endif
};

typedef void (*deca_to_cb)(uint32_t status);
typedef void (*deca_err_cb)(uint32_t status);
typedef void (*deca_rx_cb)(const struct rxbuf* buf);
typedef void (*deca_tx_complete_cb)(void);

struct txbuf {
	uint8_t buf[DWMAC_RXBUF_LEN];
	size_t len;
	bool resp;
	bool resp_multi; // multiple replies expected
	bool ranging;
	uint16_t timeout;
	int retries;
	uint64_t txtime;	 // DTU
	uint32_t rx_delay;	 // RX after TX delay in UUS
	uint16_t pto;		 // preamble detect timeout in PAC (+1)
	uint16_t tx_timeout; // maximum wait in ms
	deca_to_cb to_cb;
	deca_tx_complete_cb complete_cb;
};

bool dwmac_init(uint16_t mypanId, uint16_t myAddr, uint16_t rx_timeout_sec,
				deca_rx_cb rx_cb, deca_to_cb to_cb, deca_err_cb err_cb);
void dwmac_set_frame_filter(void);
void dwmac_set_mac64(uint64_t mac);

/* TX buffers */
struct txbuf* dwmac_txbuf_get(void);
void dwmac_txbuf_return(struct txbuf* tx);
void dwmac_tx_prepare_null(struct txbuf* tx);
void dwmac_tx_prepare(struct txbuf* tx, size_t len);
void dwmac_tx_set_ranging(struct txbuf* tx);
void dwmac_tx_expect_response(struct txbuf* tx, uint32_t delay);
void dwmac_tx_expect_multiple_responses(struct txbuf* tx);
void dwmac_tx_set_tx_timeout(struct txbuf* tx, uint16_t timeout);
void dwmac_tx_set_frame_timeout(struct txbuf* tx, uint16_t to);
void dwmac_tx_set_preamble_timeout(struct txbuf* tx, uint16_t pto);
void dwmac_tx_set_timeout_handler(struct txbuf* tx, deca_to_cb toh);
void dwmac_tx_set_complete_handler(struct txbuf* tx, void (*h)(void));
void dwmac_tx_set_txtime(struct txbuf* tx, uint64_t time);
int dwmac_tx_at_slot(struct txbuf* tx, uint8_t num_slots);
int dwmac_tx_at_slot_len(struct txbuf* tx, size_t max_pkt_len,
						 uint8_t num_slots);
bool dwmac_tx_queue(struct txbuf* tx);
bool dwmac_tx_raw(struct txbuf* tx);

void dwmac_handle_rx_frame(const struct rxbuf* rx);
void dwmac_handle_rx_timeout(uint32_t status);
void dwmac_handle_tx_done(void);
void dwmac_handle_error(uint32_t status);

void dwmac_rx_unstuck(void);

/* statistics */
void dwmac_get_cnt(uint32_t* tx_start_cnt, uint32_t* tx_irq_cnt,
				   uint32_t* rx_timout_cnt);

/** "TDMA" tx time of slot in us, pkt_len including headers */
int dwmac_get_slot_us(size_t pkt_len, int slot_num);

void dwmac_rx_reenable(void);
void dwmac_set_rx_reenable(bool b);

void deca_print_irq_status(uint32_t status);
void dwmac_print_event_counters(void);

uint16_t dwmac_get_mac16(void);
uint16_t dwmac_get_panid(void);
uint64_t dwmac_get_mac64(void);

#endif
