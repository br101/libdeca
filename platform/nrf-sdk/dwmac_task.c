/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#include "app_scheduler.h"

#include "dwmac.h"
#include "log.h"
#include "platform/dwmac_task.h"

static const char* LOG_TAG = "DWTASK";

int dwtask_init()
{
	return NRF_SUCCESS;
}

static void dwmac_sched_rx_evt(void* data, uint16_t size)
{
	struct rxbuf* rx = data;
	dwmac_handle_rx_frame(rx);
}

static void dwmac_sched_rx_timeout(void* data, uint16_t size)
{
	dwmac_handle_rx_timeout(*(uint32_t*)data);
}

static void dwmac_sched_tx_done(void* data, uint16_t size)
{
	dwmac_handle_tx_done();
}

static void dwmac_sched_error(void* data, uint16_t size)
{
	dwmac_handle_error(*(uint32_t*)data);
}

int dwtask_queue_event(enum dwevent_e type, const void* data)
{
	ret_code_t ret = NRF_ERROR_INVALID_PARAM;

	if (type == DWEVT_RX) {
		ret = app_sched_event_put(data, sizeof(struct rxbuf),
								  dwmac_sched_rx_evt);
	} else if (type == DWEVT_RX_TIMEOUT) {
		ret = app_sched_event_put(data, 4, dwmac_sched_rx_timeout);
	} else if (type == DWEVT_TX_DONE) {
		ret = app_sched_event_put(NULL, 0, dwmac_sched_tx_done);
	} else if (type == DWEVT_ERR) {
		ret = app_sched_event_put(data, 4, dwmac_sched_error);
	} else {
		LOG_ERR("Unknown event %d", type);
	}

	if (ret != NRF_SUCCESS) {
		LOG_ERR("Failed to add event to scheduler! %d", ret);
	}

	return ret;
}
