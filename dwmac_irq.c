
#include <string.h>

#include <deca_device_api.h>

#include "dwmac.h"
#include "dwtime.h"
#include "log.h"
#include "platform/dwmac_task.h"

static const char* LOG_TAG = "DECA";
extern uint32_t tx_irq_cnt;
extern struct rxbuf rx_buffer;
extern struct txbuf* current_tx;
extern bool rx_reenable;

/*** all these functions are called from dwt_isr() in interrupt context ***/

void dwmac_irq_rx_ok_cb(const dwt_cb_data_t* status)
{
	// DBG_UWB_IRQ("*** RX %lx flags %x", status->status, status->rx_flags);

#if CONFIG_DECA_DEBUG_IRQ_TIME
	uint64_t start_ts = deca_get_sys_time();
#endif

	if (current_tx != NULL && current_tx->pto != 0) {
		/* sometimes PTO triggers even though we just received a frame.
		 * this seems to happen when PTO is quite small, to avoid this
		 * we disable it here */
		// LOG_INF("*** Disable PTO");
		dwt_setpreambledetecttimeout(0);
	}

	if (status->datalength > DWMAC_RXBUF_LEN) {
		LOG_ERR_IRQ("Received frame too large");
		if (rx_reenable) {
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}
		return;
	}
	struct rxbuf* rx = &rx_buffer;
	if (rx == NULL) {
		LOG_ERR_IRQ("RX Queue full (Data)");
		if (rx_reenable) {
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}
		return;
	}

	rx->len = status->datalength;
	rx->ts = deca_get_rx_timestamp();

	if (!(status->rx_flags & DWT_CB_DATA_RX_FLAG_ND)
		&& status->datalength > 0) {
		dwt_readrxdata(rx->u.buf, status->datalength, 0);
	}

	if (status->rx_flags & DWT_CB_DATA_RX_FLAG_CPER) {
		uint16_t stat;
		dwt_readstsstatus(&stat, 0);
		if (stat & ~0x100) {
			// TODO: Don't log status "Peak growth rate warning"
			LOG_ERR_IRQ("STS error STS_TOAST: %x", stat);
		}
		// TODO: handle error
	} else if (status->rx_flags & DWT_CB_DATA_RX_FLAG_ND) {
		// TODO: STS can also be used with data frames
		int16_t stsq;
		int sts_ok = dwt_readstsquality(&stsq);
		if (!sts_ok) {
			LOG_ERR_IRQ("STS Qual not good %d", stsq);
			// TODO: handle error
		}
	}

#if DWMAC_USE_CARRIERINTEG
	rx->ci = dwt_readcarrierintegrator();
#endif

#if DWMAC_INCLUDE_RXDIAG
	dwt_readdiagnostics(&rx->diag);
#endif

	if (rx_reenable || rx->u.s.hdr.fc & MAC154_FC_FRAME_PEND
		|| (current_tx != NULL && current_tx->resp_multi)) {
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	}

#if CONFIG_DECA_DEBUG_IRQ_TIME
	rx->ts_irq_start = start_ts;
	rx->ts_irq_end = deca_get_sys_time();
#endif

	dwtask_queue_event(DWEVT_RX, rx);
}

void dwmac_irq_rx_to_cb(const dwt_cb_data_t* dat)
{
	DBG_UWB_IRQ("*** RX TO %lx", dat->status);

#if DW3000_DRIVER_VERSION >= 0x060007
	/* reset timeout values to zero, if not they keep triggering */
	if (dat->status & DWT_INT_RXFTO_BIT_MASK) {
		dwt_setrxtimeout(0);
	}
	if (dat->status & DWT_INT_RXPTO_BIT_MASK) {
		dwt_setpreambledetecttimeout(0);
	}
#else
	if (dat->status & DWT_INT_RFTO) {
		dwt_setrxtimeout(0);
	}
	if (dat->status & DWT_INT_RXPTO) {
		dwt_setpreambledetecttimeout(0);
	}
#endif

	dwtask_queue_event(DWEVT_RX_TIMEOUT, &dat->status);

	if (rx_reenable || (current_tx != NULL && current_tx->resp_multi)) {
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	}
}

void dwmac_irq_err_cb(const dwt_cb_data_t* dat)
{
	DBG_UWB_IRQ("*** ERR %x %lx", dat->rx_flags, dat->status);

	if (rx_reenable || (current_tx != NULL && current_tx->resp_multi)) {
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	}

	dwtask_queue_event(DWEVT_ERR, &dat->status);

	/* in case we are waiting for a timeout, also queue a RX_TIMEOUT event (!)
	 * so the timeout handlers are called. */
	if (current_tx != NULL
		&& (current_tx->timeout != 0 || current_tx->pto != 0)) {
		dwtask_queue_event(DWEVT_RX_TIMEOUT, &dat->status);
	}
}

void dwmac_irq_tx_done_cb(const dwt_cb_data_t* dat)
{
	// DBG_UWB_IRQ("*** TX Done %lx", dat->status);
	tx_irq_cnt++;

	if (current_tx == NULL) {
		return;
	}

	dwtask_queue_event(DWEVT_TX_DONE, NULL);
}

void dwmac_irq_spi_err_cb(const dwt_cb_data_t* dat)
{
	LOG_ERR_IRQ("*** SPI ERR");
}

void dwmac_irq_spi_rdy_cb(const dwt_cb_data_t* dat)
{
	LOG_INF_IRQ("*** SPI RDY");
	// dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_SPIRDY_BIT_MASK);
}
