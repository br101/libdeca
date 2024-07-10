/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

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
	union {
		const void* ptr;
		uint32_t status;
	} u;
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
				dwmac_handle_rx_frame(dwmac_evt.u.ptr);
				break;
			case DWEVT_RX_TIMEOUT:
				dwmac_handle_rx_timeout(dwmac_evt.u.status);
				break;
			case DWEVT_TX_DONE:
				dwmac_handle_tx_done();
				break;
			case DWEVT_ERR:
				dwmac_handle_error(dwmac_evt.u.status);
			}
		}
	}
}

int dwtask_init(void)
{
	if (dwmac_queue == NULL) {
		dwmac_queue
			= xQueueCreate(DWMAC_QUEUE_LEN, sizeof(struct dwmac_event_s));
		if (dwmac_queue == NULL) {
			LOG_ERR("Could not create queue");
			return ESP_FAIL;
		}
	}

	if (dwmac_task_hdl == NULL) {
		BaseType_t err = xTaskCreatePinnedToCore(
			dwmac_task, "dwmac_task", DWMAC_TASK_STACK_SIZE, NULL,
			DWMAC_TASK_PRIO, &dwmac_task_hdl, 0);
		if (err != pdTRUE) {
			LOG_ERR("create task failed");
			return ESP_FAIL;
		}
	}
	return ESP_OK;
}

int dwtask_queue_event(enum dwevent_e type, const void* data)
{
	struct dwmac_event_s evt = {
		.type = type,
	};

	if (type == DWEVT_RX) {
		evt.u.ptr = data;
	} else if (type == DWEVT_RX_TIMEOUT || type == DWEVT_ERR) {
		evt.u.status = *(uint32_t*)data;
	}

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
