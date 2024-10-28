/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#include "dwmac.h"
#include "platform/dwmac_task.h"
#include "log.h"

int dwtask_init()
{
	return 0;
}

// ISR is already on workqueue so we can directly call the functions
int dwtask_queue_event(enum dwevent_e type, const void* data)
{
	if (type == DWEVT_RX) {
		const struct rxbuf* rx = data;
		dwmac_handle_rx_frame(rx);
	} else if (type == DWEVT_RX_TIMEOUT) {
		dwmac_handle_rx_timeout(*(uint32_t*)data);
	} else if (type == DWEVT_TX_DONE) {
		dwmac_handle_tx_done();
	} else if (type == DWEVT_ERR) {
		dwmac_handle_error(*(uint32_t*)data);
	} else {
		LOG_ERR("Unknown event %d", type);
	}

	return 0;
}
