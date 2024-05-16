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
	dwmac_handle_rx_event(rx);
}

static void dwmac_sched_tx_done(void* data, uint16_t size)
{
	dwmac_handle_tx_done_event();
}

int dwtask_queue_event(enum dwevent_e type, void* data)
{
	ret_code_t ret = NRF_ERROR_INVALID_PARAM;

	if (type == DWEVT_RX) {
		ret = app_sched_event_put(data, sizeof(struct rxbuf),
								  dwmac_sched_rx_evt);
	} else if (type == DWEVT_TX_DONE) {
		ret = app_sched_event_put(NULL, 0, dwmac_sched_tx_done);
	} else {
		LOG_ERR("Unknown event");
	}

	if (ret != NRF_SUCCESS) {
		LOG_ERR("Failed to add event to scheduler! %ld", ret);
	}

	return ret;
}
