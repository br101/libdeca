#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "dw3000_hw.h"
#include "dwhw.h"
#include "dwmac.h"
#include "dwmac_task.h"
#include "dwphy.h"
#include "log.h"
#include "ranging.h"

#define DWMAC_TASK_STACK_SIZE 4096 // TODO
#define DWMAC_TASK_PRIO		  5	   // TODO
#define DWMAC_QUEUE_LEN		  10

struct dwmac_event_s {
	enum dwevent_e type;
	void* data;
};

static const char* LOG_TAG = "DWTASK";
static TaskHandle_t dwmac_task_hdl;
static QueueHandle_t dwmac_queue;
static struct dwmac_event_s dwmac_evt;

static void dwmac_task(void* pvParameters)
{
	BaseType_t rc;
	LOG_INF("TASK started");

	while (true) {
		rc = xQueueReceive(dwmac_queue, &dwmac_evt, portMAX_DELAY);
		if (rc == pdTRUE) { // received
			switch (dwmac_evt.type) {
			case DWEVT_RX:
				struct rxbuf* rx = dwmac_evt.data;
				dwmac_handle_rx_event(rx);
				break;
			case DWEVT_TX_DONE:
				dwmac_handle_tx_done_event();
				break;
			}
		}
	}
}

int dwtask_init(void)
{
	dwmac_queue = xQueueCreate(DWMAC_QUEUE_LEN, sizeof(struct dwmac_event_s));
	if (dwmac_queue == NULL) {
		LOG_ERR("Could not create queue");
		return ESP_FAIL;
	}

	BaseType_t err = xTaskCreatePinnedToCore(
		dwmac_task, "dwmac_task", DWMAC_TASK_STACK_SIZE, NULL, DWMAC_TASK_PRIO,
		&dwmac_task_hdl, 0);
	if (err != pdTRUE) {
		LOG_ERR("create task failed");
		return ESP_FAIL;
	}
	return ESP_OK;
}

int dwtask_queue_event(enum dwevent_e type, void* data)
{
	struct dwmac_event_s evt = {
		.type = type,
		.data = data,
	};

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t rc;

	rc = xQueueSendToBackFromISR(dwmac_queue, &evt, &xHigherPriorityTaskWoken);
	if (rc != pdTRUE) {
		LOG_ERR("Queue failed");
		return ESP_FAIL;
	}

	if (xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR();
	}

	return ESP_OK;
}
