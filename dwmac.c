/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#include <stdlib.h>
#include <string.h>

#include <deca_device_api.h>
#include <deca_version.h>
#ifdef DW3000_DRIVER_VERSION // == 0x040000
#include <deca_regs.h>
#endif

#include "dwhw.h"
#include "dwmac.h"
#include "dwphy.h"
#include "dwtime.h"
#include "dwutil.h"
#include "mac802154.h"
#include "platform/dwmac_task.h"

#include "log.h"

#define SLOT_SPI_TIME(_x) (_x * 2.7) /* us TODO: old */
#define SLOT_PROC_TIME	  500		 /* TODO: now used with systime: only TX */
#define SLOT_GAP		  5

#ifndef __ZEPHYR__
static const char* LOG_TAG = "DECA";
#endif
static uint16_t panId;
static uint16_t macAddr;
static uint64_t mac64;

static deca_rx_cb dwmac_rx_cb = NULL;
static deca_to_cb dwmac_to_cb = NULL;
static deca_err_cb dwmac_err_cb = NULL;
static uint32_t mac_tx_cnt = 0;
static struct txbuf tx_buffer;

/* shared with dwmac_irq.c */
struct rxbuf rx_buffer;
struct txbuf* current_tx = NULL;
bool rx_reenable = false;

extern void dwmac_irq_rx_ok_cb(const dwt_cb_data_t* dat);
extern void dwmac_irq_rx_to_cb(const dwt_cb_data_t* dat);
extern void dwmac_irq_err_cb(const dwt_cb_data_t* dat);
extern void dwmac_irq_tx_done_cb(const dwt_cb_data_t* dat);
extern void dwmac_irq_spi_err_cb(const dwt_cb_data_t* dat);
extern void dwmac_irq_spi_rdy_cb(const dwt_cb_data_t* dat);

#if DRIVER_VERSION_HEX >= 0x080202
dwt_callbacks_s dwmac_cbs = {
	.cbTxDone = dwmac_irq_tx_done_cb,
	.cbRxOk = dwmac_irq_rx_ok_cb,
	.cbRxTo = dwmac_irq_rx_to_cb,
	.cbRxErr = dwmac_irq_err_cb,
	.cbSPIErr = dwmac_irq_spi_err_cb,
	.cbSPIRDErr = NULL,
	.cbSPIRdy = dwmac_irq_spi_rdy_cb,
	.cbDualSPIEv = NULL,
	.cbFrmRdy = NULL,
	.cbCiaDone = NULL,
	.devErr = NULL,
	.cbSysEvent = NULL,
};
#endif

bool dwmac_init(uint16_t mypanId, uint16_t myAddr, deca_rx_cb rx_cb,
				deca_to_cb to_cb, deca_err_cb err_cb)
{
	ASSERT_RET(myAddr != 0 && myAddr != 0xffff);

	panId = mypanId;
	macAddr = myAddr;
	rx_reenable = false;
	dwmac_rx_cb = rx_cb;
	dwmac_to_cb = to_cb;
	dwmac_err_cb = err_cb;

	LOG_INF("Init PANID: " ADDR_FMT " MAC: " ADDR_FMT, panId, macAddr);

	/* frame filtering OFF */
	dwt_setaddress16(macAddr);
	dwt_setpanid(panId);
	dwt_configureframefilter(DWT_FF_DISABLE, 0);

	/* for 'dwcnt' command */
	dwt_configeventcounters(1);

#if DRIVER_VERSION_HEX >= 0x080202

	dwt_setcallbacks(&dwmac_cbs);
	dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK | DWT_INT_TXFRS_BIT_MASK
						 | DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK
						 | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXFTO_BIT_MASK
						 | DWT_INT_CIAERR_BIT_MASK | DWT_INT_RXPTO_BIT_MASK
						 | DWT_INT_RXSTO_BIT_MASK
#if CONFIG_DECA_DEBUG_FRAME_FILTER
						 | DWT_INT_ARFE_BIT_MASK
#endif
					 ,
					 0, DWT_ENABLE_INT_ONLY);

#elif DRIVER_VERSION_HEX >= 0x060007

	dwt_setcallbacks(dwmac_irq_tx_done_cb, dwmac_irq_rx_ok_cb,
					 dwmac_irq_rx_to_cb, dwmac_irq_err_cb, dwmac_irq_spi_err_cb,
					 dwmac_irq_spi_rdy_cb, NULL);

	dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK | DWT_INT_TXFRS_BIT_MASK
						 | DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK
						 | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXFTO_BIT_MASK
						 | DWT_INT_CIAERR_BIT_MASK | DWT_INT_RXPTO_BIT_MASK
						 | DWT_INT_RXSTO_BIT_MASK
#if CONFIG_DECA_DEBUG_FRAME_FILTER
						 | DWT_INT_ARFE_BIT_MASK
#endif
					 ,
					 0, DWT_ENABLE_INT_ONLY);

#else

	dwt_setcallbacks(dwmac_irq_tx_done_cb, dwmac_irq_rx_ok_cb,
					 dwmac_irq_rx_to_cb, dwmac_irq_err_cb, dwmac_irq_spi_err_cb,
					 dwmac_irq_spi_rdy_cb);

	dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_TFRS | DWT_INT_RPHE | DWT_INT_RFCE
						 | DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_LDEERR
						 | DWT_INT_RXPTO | DWT_INT_SFDT
#if CONFIG_DECA_DEBUG_FRAME_FILTER
						 | DWT_INT_ARFE
#endif
					 ,
					 0, DWT_ENABLE_INT_ONLY);
#endif

	dwtask_init();

	return true;
}

void dwmac_set_frame_filter(void)
{
	dwt_configureframefilter(DWT_FF_ENABLE_802_15_4,
							 DWT_FF_BEACON_EN | DWT_FF_DATA_EN | DWT_FF_ACK_EN
								 | DWT_FF_COORD_EN);
}

void deca_print_sys_status(uint32_t status)
{
#ifdef DRIVER_VERSION_HEX // >= 0x060007
	if (status & DWT_INT_TIMER1_BIT_MASK) {
		LOG_INF("- TIMER1 expiry");
	}
	if (status & DWT_INT_TIMER0_BIT_MASK) {
		LOG_INF("- TIMER0 expiry");
	}
	if (status & DWT_INT_ARFE_BIT_MASK) {
		LOG_INF("- Frame filtered");
	}
	if (status & DWT_INT_CPERR_BIT_MASK) {
		LOG_INF("- STS quality warning/error");
	}
	if (status & DWT_INT_HPDWARN_BIT_MASK) {
		LOG_INF("- Half period warning");
	}
	if (status & DWT_INT_RXSTO_BIT_MASK) {
		LOG_INF("- SFD timeout");
	}
	if (status & DWT_INT_PLL_HILO_BIT_MASK) {
		LOG_INF("- Clock PLL Losing Lock");
	}
	if (status & DWT_INT_RCINIT_BIT_MASK) {
		LOG_INF("- Device has entered IDLE_RC");
	}
	if (status & DWT_INT_SPIRDY_BIT_MASK) {
		LOG_INF("- SPI ready");
	}
	if (status & DWT_INT_RXPTO_BIT_MASK) {
		LOG_INF("- RX Preamble timeout");
	}
	if (status & DWT_INT_RXOVRR_BIT_MASK) {
		LOG_INF("- RX overrun (double RX buffer)");
	}
	if (status & DWT_INT_VWARN_BIT_MASK) {
		LOG_INF("- Low voltage warning");
	}
	if (status & DWT_INT_CIAERR_BIT_MASK) {
		LOG_INF("- CIA error");
	}
	if (status & DWT_INT_RXFTO_BIT_MASK) {
		LOG_INF("- RX frame wait timeout");
	}
	if (status & DWT_INT_RXFSL_BIT_MASK) {
		LOG_INF("- RX Reed-Solomon sync loss");
	}
	if (status & DWT_INT_RXFCE_BIT_MASK) {
		LOG_INF("- RX frame CRC error");
	}
	if (status & DWT_INT_RXFCG_BIT_MASK) {
		LOG_INF("- RX frame CRC good");
	}
	if (status & DWT_INT_RXFR_BIT_MASK) {
		LOG_INF("- RX data frame ready");
	}
	if (status & DWT_INT_RXPHE_BIT_MASK) {
		LOG_INF("- RX PHY header error");
	}
	if (status & DWT_INT_RXPHD_BIT_MASK) {
		LOG_INF("- RX PHY header detected");
	}
	if (status & DWT_INT_CIADONE_BIT_MASK) {
		LOG_INF("- CIA done");
	}
	if (status & DWT_INT_RXSFDD_BIT_MASK) {
		LOG_INF("- RX SFD detected");
	}
	if (status & DWT_INT_RXPRD_BIT_MASK) {
		LOG_INF("- RX Preamble detected");
	}
	if (status & DWT_INT_TXFRS_BIT_MASK) {
		LOG_INF("- TX Frame sent");
	}
	if (status & DWT_INT_TXPHS_BIT_MASK) {
		LOG_INF("- TX PHR sent");
	}
	if (status & DWT_INT_TXPRS_BIT_MASK) {
		LOG_INF("- TX preamble sent");
	}
	if (status & DWT_INT_TXFRB_BIT_MASK) {
		LOG_INF("- TX frame begins");
	}
	if (status & DWT_INT_AAT_BIT_MASK) {
		LOG_INF("- Automatic ACK transmission pending");
	}
	if (status & DWT_INT_SPICRCE_BIT_MASK) {
		LOG_INF("- SPI CRC error");
	}
	if (status & DWT_INT_CP_LOCK_BIT_MASK) {
		LOG_INF("- Clock PLL locked");
	}
	// if (status & DWT_INT_IRQS_BIT_MASK) {
	//	LOG_INF("- Interrupt set");
	// }
#else

	if (status & SYS_STATUS_ARFE_BIT_MASK) {
		LOG_INF("- RX frame filtered");
	}
	if (status & SYS_STATUS_CPERR_BIT_MASK) {
		LOG_INF("- STS quality warning/error");
	}
	if (status & SYS_STATUS_HPDWARN_BIT_MASK) {
		LOG_INF("- Half period warning flag when delayed TX/RX is used");
	}
	if (status & SYS_STATUS_RXSTO_BIT_MASK) {
		LOG_INF("- SFD timeout");
	}
	if (status & SYS_STATUS_PLL_HILO_BIT_MASK) {
		LOG_INF("- Clock PLL Losing Lock");
	}
	if (status & SYS_STATUS_RCINIT_BIT_MASK) {
		LOG_INF("- Device has entered IDLE_RC");
	}
	if (status & SYS_STATUS_SPIRDY_BIT_MASK) {
		LOG_INF("- SPI ready");
	}
	if (status & SYS_STATUS_RXPTO_BIT_MASK) {
		LOG_INF("- RX Preamble timeout");
	}
	if (status & SYS_STATUS_RXOVRR_BIT_MASK) {
		LOG_INF("- RX overrun (double RX buffer)");
	}
	if (status & SYS_STATUS_VWARN_BIT_MASK) {
		LOG_INF("- Low voltage warning");
	}
	if (status & SYS_STATUS_CIAERR_BIT_MASK) {
		LOG_INF("- CIA error");
	}
	if (status & SYS_STATUS_RXFTO_BIT_MASK) {
		LOG_INF("- RX frame wait timeout");
	}
	if (status & SYS_STATUS_RXFSL_BIT_MASK) {
		LOG_INF("- RX Reed-Solomon sync loss");
	}
	if (status & SYS_STATUS_RXFCE_BIT_MASK) {
		LOG_INF("- RX frame CRC error");
	}
	if (status & SYS_STATUS_RXFCG_BIT_MASK) {
		LOG_INF("- RX frame CRC good");
	}
	if (status & SYS_STATUS_RXFR_BIT_MASK) {
		LOG_INF("- RX data frame ready");
	}
	if (status & SYS_STATUS_RXPHE_BIT_MASK) {
		LOG_INF("- RX PHY header error");
	}
	if (status & SYS_STATUS_RXPHD_BIT_MASK) {
		LOG_INF("- RX PHY header detected");
	}
	if (status & SYS_STATUS_CIADONE_BIT_MASK) {
		LOG_INF("- CIA done");
	}
	if (status & SYS_STATUS_RXSFDD_BIT_MASK) {
		LOG_INF("- RX SFD detected");
	}
	if (status & SYS_STATUS_RXPRD_BIT_MASK) {
		LOG_INF("- RX Preamble detected");
	}
	if (status & SYS_STATUS_TXFRS_BIT_MASK) {
		LOG_INF("- TX Frame sent");
	}
	if (status & SYS_STATUS_TXPHS_BIT_MASK) {
		LOG_INF("- TX PHR sent");
	}
	if (status & SYS_STATUS_TXPRS_BIT_MASK) {
		LOG_INF("- TX preamble sent");
	}
	if (status & SYS_STATUS_TXFRB_BIT_MASK) {
		LOG_INF("- TX frame begins");
	}
	if (status & SYS_STATUS_AAT_BIT_MASK) {
		LOG_INF("- Automatic ACK transmission pending");
	}
	if (status & SYS_STATUS_SPICRCE_BIT_MASK) {
		LOG_INF("- SPI CRC error");
	}
	if (status & SYS_STATUS_CP_LOCK_BIT_MASK) {
		LOG_INF("- Clock PLL locked");
	}
	// if (status & SYS_STATUS_IRQS_BIT_MASK) {
	//	LOG_INF("- Interrupt set");
	// }
#endif
}

struct txbuf* dwmac_txbuf_get(void)
{
	// there is only one TX buffer for now
	return &tx_buffer;
}

void dwmac_txbuf_return(struct txbuf* tx)
{
	// there is only one TX buffer for now
}

void dwmac_tx_prepare_null(struct txbuf* tx)
{
	tx->len = 0;
	tx->resp = false;
	tx->resp_multi = false;
	tx->ranging = false;
	tx->sleep_after_tx = false;
	tx->rx_timeout = 0;
	tx->txtime = 0;
	tx->rx_delay = 0;
	tx->pto = 0;
	tx->to_cb = NULL;
	tx->complete_cb = NULL;
}

/* len is without FCS */
void dwmac_tx_prepare(struct txbuf* tx, size_t len)
{
	dwmac_tx_prepare_null(tx);
	tx->len = len + MAC154_FCS_LEN;
}

void dwmac_tx_set_ranging(struct txbuf* tx)
{
	tx->ranging = true;
}

/* expect response (RX on) after delay (may be 0) in UUS */
void dwmac_tx_expect_response(struct txbuf* tx, uint32_t delay)
{
	tx->resp = true;
	tx->rx_delay = delay;
}

/* expect multiple responses (turn RX on after receive) */
void dwmac_tx_expect_multiple_responses(struct txbuf* tx)
{
	tx->resp = true;
	tx->resp_multi = true;
}

/** timeout in UUS, implies response expected */
void dwmac_tx_set_rx_timeout(struct txbuf* tx, uint16_t to)
{
	tx->resp = true;
	tx->rx_timeout = to;
}

/* timeout in units of PAC size, implies response expected */
void dwmac_tx_set_preamble_timeout(struct txbuf* tx, uint16_t pto)
{
	tx->resp = true;
	tx->pto = pto;
}

void dwmac_tx_set_sleep_after_tx(struct txbuf* tx)
{
	tx->sleep_after_tx = true;
}

void dwmac_tx_set_timeout_handler(struct txbuf* tx, deca_to_cb toh)
{
	tx->to_cb = toh;
}

void dwmac_tx_set_complete_handler(struct txbuf* tx, deca_tx_complete_cb h)
{
	tx->complete_cb = h;
}

/* time resolution of tx time is 8ns: given in DTU with last 9 bits 0 */
void dwmac_tx_set_txtime(struct txbuf* tx, uint64_t time)
{
	tx->txtime = time;
}

bool dwmac_tx_raw(struct txbuf* tx)
{
	int ret;

	decaIrqStatus_t stat = decamutexon();

	/* make sure we're out of RX mode before initiating TX */
	dwt_forcetrxoff();

	if (tx->sleep_after_tx) {
		dwhw_enable_tx_interrupt(false);
		dwt_entersleepaftertx(1);
	}

	if (tx->len > 0) {
		ret = dwt_writetxdata(tx->len, tx->buf, 0);
		if (ret != DWT_SUCCESS) {
			return false;
		}
		// NOTE: this is not necessary to set if we use STS_MODE_ND
		dwt_writetxfctrl(tx->len, 0, tx->ranging);
	}

	dwt_setrxtimeout(tx->rx_timeout);
	dwt_setrxaftertxdelay(tx->rx_delay);
	dwt_setpreambledetecttimeout(tx->pto);

	if (tx->txtime) {
		dwt_setdelayedtrxtime(DTU_TO_DELAYEDTRX(tx->txtime));
	}

	mac_tx_cnt++;
	ret = dwt_starttx(
		(tx->txtime ? DWT_START_TX_DELAYED : DWT_START_TX_IMMEDIATE)
		| (tx->resp || rx_reenable ? DWT_RESPONSE_EXPECTED : 0));

	decamutexoff(stat);

	// plat_led_act_trigger();

	if (ret != DWT_SUCCESS) {
		LOG_ERR("TX error (%p)", tx);
		if (tx->txtime) {
			uint64_t systime = dw_get_systime();
			LOG_ERR_TS("\tSYS Time:\t", systime);
			LOG_ERR_TS("\tTX Time:\t", tx->txtime);
			int diff = tx->txtime - systime;
			LOG_ERR("\tDiff:\t\t%x (%d us)", diff, (int)DTU_TO_US(diff));
			/* decadriver disables TRX in the error case */
			if (rx_reenable)
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
			return false;
		}
	}

#if CONFIG_DECA_DEBUG_TX_TIME
	if (tx->txtime) {
		uint64_t systime = deca_get_sys_time();
		LOG_DBG("Delayed TX in %d us (%p)",
				(int)DTU_TO_US(tx->txtime - systime), tx);
	} else {
		LOG_DBG("TX (%p)", tx);
	}
#endif

#if CONFIG_DECA_DEBUG_TX_DUMP
	LOG_HEXDUMP("TX", tx->buf, tx->len);
#endif

	if (tx->sleep_after_tx) {
		dwhw_sleep_after_tx();
	}

	return true;
}

bool dwmac_transmit(struct txbuf* tx)
{
	if (tx == NULL) {
		LOG_ERR("TX invalid");
		return false;
	}

	current_tx = tx;
	return dwmac_tx_raw(tx);
}

void dwmac_handle_rx_frame(const struct rxbuf* rx)
{
#if CONFIG_DECA_DEBUG_IRQ_TIME
	uint64_t st = dw_get_systime();
	LOG_INF("RX to CB:\t%d us", (int)DTU_TO_US(rx->ts_irq_start - rx->ts));
	LOG_INF("RX to Sched:\t%d us", (int)DTU_TO_US(st - rx->ts));
	LOG_INF("Time in CB:\t%d us",
			(int)DTU_TO_US(rx->ts_irq_end - rx->ts_irq_start));
	LOG_DBG("IRQ start %" PRIu32, (uint32_t)rx->ts_irq_start);
	LOG_DBG("IRQ end %" PRIu32, (uint32_t)rx->ts_irq_end);
#endif

#if CONFIG_DECA_DEBUG_RX_DUMP
	LOG_HEXDUMP("RX", rx->buf, rx->len);
#endif

#if CONFIG_DECA_READ_RXDIAG
	LOG_INF("DIAG preamb %d", rx->diag.rxPreamCount);
#endif

#if CONFIG_DECA_XTAL_TRIM
	if (rx->len > 0) {
		dwphy_xtal_trim();
	}
#endif

	if (dwmac_rx_cb != NULL) {
		dwmac_rx_cb(rx);
	}
}

void dwmac_handle_rx_timeout(uint32_t status)
{
#if CONFIG_DECA_DEBUG_RX_STATUS
	deca_print_sys_status(status);
#endif

	if (current_tx && current_tx->to_cb != NULL) {
		current_tx->to_cb(status);
	}

	if (dwmac_to_cb != NULL) {
		dwmac_to_cb(status);
	}
}

void dwmac_handle_tx_done(void)
{
	// callback after removing current_tx so we can TX again
	deca_tx_complete_cb cb = NULL;
	if (current_tx != NULL && current_tx->complete_cb != NULL) {
		cb = current_tx->complete_cb;
	}

	// only remove TX if we are not waiting for a RX timeout
	if (current_tx != NULL && current_tx->pto == 0
		&& current_tx->rx_timeout == 0) {
		current_tx = NULL;
	}

	if (cb) {
		cb();
	}
}

void dwmac_cleanup_sleep_after_tx(void)
{
	if (current_tx != NULL && current_tx->sleep_after_tx) {
		dwt_entersleepaftertx(0); // Disable Sleep after TX!
		dwhw_enable_tx_interrupt(true);
		dwmac_handle_tx_done();
	}
}

void dwmac_handle_error(uint32_t status)
{
#if CONFIG_DECA_DEBUG_RX_STATUS | CONFIG_DECA_DEBUG_FRAME_FILTER
	deca_print_sys_status(status);
#endif

	if (dwmac_err_cb != NULL) {
		dwmac_err_cb(status);
	}
}

/* return tx delay in usec for slot (starting from 1) */
int dwmac_get_slot_us(size_t pkt_len, int slot_num)
{
	ASSERT_RET(slot_num > 0);
	ASSERT_RET(pkt_len > 0);

	uint8_t plen = dwphy_get_plen();
	uint8_t prf = dwphy_get_prf();
	uint8_t rate = dwphy_get_rate();

	/* first delay is equal for all slots */
	int delay = SLOT_PROC_TIME; // constant processing time for RX and TX
	delay += SLOT_SPI_TIME(pkt_len);
	delay += PKTTIME_TO_USEC(dwphy_calc_preamble_time(plen, prf, rate));

	/* then multiply slot duration by slot number */
	slot_num--;
	int slot_duration
		= PKTTIME_TO_USEC(dwphy_calc_packet_time(rate, plen, prf, pkt_len));
	slot_duration += SLOT_GAP;
	delay += slot_duration * slot_num;

	LOG_DBG("Slot %d = %d for %dB", slot_num, delay, pkt_len);
	return delay;
}

void dwmac_rx_reenable(void)
{
	if (rx_reenable) {
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	}
}

void dwmac_set_rx_reenable(bool b)
{
	if (b && !rx_reenable) {
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	} else if (!b && rx_reenable) {
		dwt_forcetrxoff();
	}

	rx_reenable = b;
}

void dwmac_print_event_counters(void)
{
	dwt_deviceentcnts_t counters;
	dwt_readeventcounters(&counters);

	LOG_INF("PHE   %d (header error)", counters.PHE);
	LOG_INF("RSL   %d (frame sync loss)", counters.RSL);
	LOG_INF("CRCG  %d (good CRC)", counters.CRCG);
	LOG_INF("CRCB  %d (CRC error)", counters.CRCB);
	LOG_INF("ARFE  %d (address filter)", counters.ARFE);
	LOG_INF("OVER  %d (receive buffer overrun)", counters.OVER);
	LOG_INF("SFDTO %d (SFD timeout)", counters.SFDTO);
	LOG_INF("PTO   %d (Preamble timeout)", counters.PTO);
	LOG_INF("RTO   %d (RX frame wait timeout)", counters.RTO);
	LOG_INF("TXF   %d (transmitted frame)", counters.TXF);
	LOG_INF("HPW   %d (half period warning)", counters.HPW);
	LOG_INF("CRCE  %d (SPI CRC error)", counters.CRCE);
	LOG_INF("PREJ  %d (Preamble rejection)", counters.PREJ);
#ifdef DRIVER_VERSION_HEX // >= 0x060007
	LOG_INF("SFDD  %d (SFD detection)", counters.SFDD);
	LOG_INF("STSE  %d (STS error/warning)", counters.STSE);
#endif
}

uint16_t dwmac_get_mac16(void)
{
	return macAddr;
}

uint16_t dwmac_get_panid(void)
{
	return panId;
}

uint64_t dwmac_get_mac64(void)
{
	return mac64;
}

void dwmac_set_mac64(uint64_t mac)
{
	mac64 = mac;
	dwt_seteui((uint8_t*)&mac64);
}

uint32_t dwmac_get_tx_start_cnt(void)
{
	return mac_tx_cnt;
}
