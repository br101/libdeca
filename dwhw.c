/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#include <deca_device_api.h>
#include <deca_version.h>
#include <dw3000_hw.h>
#include <dw3000_spi.h>

#include "dwhw.h"
#include "dwmac.h"
#include "log.h"

#ifdef DRIVER_VERSION_HEX // >= 0x060007
extern const struct dwt_probe_s dw3000_probe_interf;
#endif

static const char* LOG_TAG = "DECA";

static bool dwchip_ready = false;

bool dwhw_init(void)
{
	int ret;

#ifdef DRIVER_VERSION_HEX
	LOG_INF("Decadriver lib: " DRIVER_VERSION_STR);
#else
	LOG_INF("Decadriver source: " DW3000_DEVICE_DRIVER_VER_STRING);
#endif

#ifdef DRIVER_VERSION_HEX // >= 0x060007
	ret = dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf);
	if (ret < 0) {
		LOG_ERR("DWT Probe failed");
		return false;
	}
#endif

	uint32_t dev_id = dwt_readdevid();
	ret = dwt_check_dev_id();
	if (ret != DWT_SUCCESS) {
		LOG_ERR("UNKNOWN DEV ID: %lX", dev_id);
		return false;
	} else {
		LOG_INF("DEV ID: %lX", dev_id);
	}

	int cnt = 1000;
	while (!dwt_checkidlerc() && cnt-- > 0) {
		deca_sleep(1);
	}
	if (cnt <= 0) {
		LOG_ERR("Init did not leave IDLE state");
		return false;
	}

	// dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

	if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
		LOG_ERR("INIT failed");
		return false;
	}

	dw3000_spi_speed_fast();
	dwchip_ready = true;
	return true;
}

/** Note: While in DEEPSLEEP power should not be applied to GPIO, SPICLK or
	SPIMISO pins as this will cause an increase in leakage current */
void dwhw_sleep(void)
{
	if (!dwchip_ready) {
		LOG_ERR("not present or already sleeping");
		return;
	}

	LOG_INF("Sleep");
	dwchip_ready = false;

	/* pull WAKEUP line low, later we will use it for waking up */
	dw3000_hw_wakeup_pin_low();

	/* Sleep configuration:
	 * - wakeup on CS pin
	 * - wakeup on WAKEUP
	 * - enable sleep mode */
	dwt_configuresleep(DWT_CONFIG, DWT_SLP_EN | DWT_WAKE_CSN | DWT_WAKE_WUP);
	dwt_entersleep(DWT_DW_IDLE);

	/* While in DEEPSLEEP power should not be applied to GPIO, SPICLK or
	SPIMISO pins as this will cause an increase in leakage current */
	dw3000_spi_fini();
}

bool dwhw_wakeup(void)
{
	if (dwchip_ready) {
		LOG_ERR("Already woke up?");
		return false;
	}

	dw3000_spi_reinit();
	dw3000_hw_wakeup();

	dwt_restoreconfig();

	int ret = dwt_check_dev_id();
	if (ret != DWT_SUCCESS) {
		LOG_ERR("Failed to read device ID after wakeup!");
		return false;
	}

	/* Wait for device to become ready (IDLE state) */
	int cnt = 1000;
	while (!dwt_checkidlerc() && cnt-- > 0) {
		deca_sleep(1);
	}
	if (cnt <= 0) {
		LOG_ERR("Wakeup did not leave IDLE state");
		return false;
	}

	dw3000_spi_speed_fast();
	dwmac_cleanup_sleep_after_tx();
	dwchip_ready = true;
	return true;
}

bool dwhw_is_ready(void)
{
	return dwchip_ready;
}

void dwhw_sleep_after_tx(void)
{
	dwchip_ready = false;
	dw3000_spi_fini();
}
