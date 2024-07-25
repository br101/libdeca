/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#ifndef DECA_MAC_H
#define DECA_MAC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <deca_device_api.h>

#if ESP_PLATFORM
#include <sdkconfig.h>
#endif

/* Read the carrier integrator value for each received frame and include it in
 * the RX buffer */
#ifndef CONFIG_DECA_USE_CARRIERINTEG
#define CONFIG_DECA_USE_CARRIERINTEG 0
#endif

/* Read the RX diag structure and include it in the RX buffer */
#ifndef CONFIG_DECA_READ_RXDIAG
#define CONFIG_DECA_READ_RXDIAG 0
#endif

/* Perform XTAL trimming, adjusting the local clock to the clock offset of
 * another sender. Careful! This can reduce reception of other senders with a
 * different clock offset! */
#ifndef CONFIG_DECA_XTAL_TRIM
#define CONFIG_DECA_XTAL_TRIM 0
#endif

/* Debugging configs */

#ifndef CONFIG_DECA_DEBUG_IRQ_TIME
#define CONFIG_DECA_DEBUG_IRQ_TIME 0
#endif

#ifndef CONFIG_DECA_DEBUG_RX_DUMP
#define CONFIG_DECA_DEBUG_RX_DUMP 0
#endif

#ifndef CONFIG_DECA_DEBUG_RX_STATUS
#define CONFIG_DECA_DEBUG_RX_STATUS 0
#endif

#ifndef CONFIG_DECA_DEBUG_TX_DUMP
#define CONFIG_DECA_DEBUG_TX_DUMP 0
#endif

#ifndef CONFIG_DECA_DEBUG_TX_TIME
#define CONFIG_DECA_DEBUG_TX_TIME 0
#endif

#ifndef CONFIG_DECA_DEBUG_FRAME_FILTER
#define CONFIG_DECA_DEBUG_FRAME_FILTER 0
#endif

#ifndef CONFIG_DECA_DEBUG_OUTPUT_IRQ
#define CONFIG_DECA_DEBUG_OUTPUT_IRQ 0
#endif

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
#if CONFIG_DECA_USE_CARRIERINTEG
	int32_t ci; /* carrier integrator for clock offset */
#endif
#if CONFIG_DECA_READ_RXDIAG
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
	bool resp;			 // response expected
	bool resp_multi;	 // multiple responses expected
	bool ranging;		 // ranging
	bool sleep_after_tx; // goto sleep after TX
	uint16_t rx_timeout; // RX timeout
	uint64_t txtime;	 // DTU
	uint32_t rx_delay;	 // RX after TX delay in UUS
	uint16_t pto;		 // preamble detect timeout in PAC (+1)
	deca_to_cb to_cb;
	deca_tx_complete_cb complete_cb;
};

bool dwmac_init(uint16_t mypanId, uint16_t myAddr, deca_rx_cb rx_cb,
				deca_to_cb to_cb, deca_err_cb err_cb);
void dwmac_set_frame_filter(void);
void dwmac_set_mac64(uint64_t mac);

/* TX buffers */
struct txbuf* dwmac_txbuf_get(void);
void dwmac_txbuf_return(struct txbuf* tx);
void dwmac_tx_prepare_null(struct txbuf* tx);
void dwmac_tx_prepare(struct txbuf* tx, size_t len);
void dwmac_tx_set_ranging(struct txbuf* tx);
void dwmac_tx_set_txtime(struct txbuf* tx, uint64_t time);
void dwmac_tx_expect_response(struct txbuf* tx, uint32_t delay);
void dwmac_tx_expect_multiple_responses(struct txbuf* tx);
void dwmac_tx_set_rx_timeout(struct txbuf* tx, uint16_t to);
void dwmac_tx_set_preamble_timeout(struct txbuf* tx, uint16_t pto);
void dwmac_tx_set_sleep_after_tx(struct txbuf* tx);
void dwmac_tx_set_timeout_handler(struct txbuf* tx, deca_to_cb toh);
void dwmac_tx_set_complete_handler(struct txbuf* tx, void (*h)(void));
bool dwmac_transmit(struct txbuf* tx);

void dwmac_cleanup_sleep_after_tx(void);

void dwmac_rx_reenable(void);
void dwmac_set_rx_reenable(bool b);

void deca_print_sys_status(uint32_t status);
void dwmac_print_event_counters(void);

uint16_t dwmac_get_mac16(void);
uint16_t dwmac_get_panid(void);
uint64_t dwmac_get_mac64(void);

/** "TDMA" tx time of slot in us, pkt_len including headers */
int dwmac_get_slot_us(size_t pkt_len, int slot_num);

/* statistics */
uint32_t dwmac_get_tx_start_cnt(void);
uint32_t dwmac_get_tx_done_cnt(void); // dwmac_irq.c

/* INTERNAL: called from task / scheduler context */
void dwmac_handle_rx_frame(const struct rxbuf* rx);
void dwmac_handle_rx_timeout(uint32_t status);
void dwmac_handle_tx_done(void);
void dwmac_handle_error(uint32_t status);

#endif
