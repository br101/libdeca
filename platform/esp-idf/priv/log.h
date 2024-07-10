/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#include <esp_log.h>
#include <esp_attr.h>

#define LOG_ERR(...)  ESP_LOGE(LOG_TAG, __VA_ARGS__)
#define LOG_WARN(...) ESP_LOGW(LOG_TAG, __VA_ARGS__)
#define LOG_INF(...)  ESP_LOGI(LOG_TAG, __VA_ARGS__)
#define LOG_DBG(...)  ESP_LOGD(LOG_TAG, __VA_ARGS__)

#define LOG_HEXDUMP(...) ESP_LOG_BUFFER_HEX(__VA_ARGS__)

#define DBG_UWB(...) ESP_LOGD(LOG_TAG, __VA_ARGS__)

#define LOG_INF_IRQ(...) ESP_DRAM_LOGI(LOG_TAG, __VA_ARGS__)
#define LOG_ERR_IRQ(...) ESP_DRAM_LOGE(LOG_TAG, __VA_ARGS__)
#define DBG_UWB_IRQ(...) ESP_DRAM_LOGI(LOG_TAG, __VA_ARGS__)
