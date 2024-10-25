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
#ifdef DW3000_DRIVER_VERSION // == 0x040000
#include <deca_regs.h>
#endif
#include <dw3000_hw.h>
#include <dw3000_spi.h>

#include "dwhw.h"
#include "dwmac.h"
#include "log.h"

#define DWHW_DEBUG_WAKEUP 0

#ifdef DRIVER_VERSION_HEX // >= 0x060007
extern const struct dwt_probe_s dw3000_probe_interf;
#endif

static const char* LOG_TAG = "DECA";
static bool dwchip_ready = false;
static float last_calib_temp;

static void dwhw_update_calib_temp(void)
{
	uint16_t tv = dwt_readtempvbat();
	last_calib_temp = dwt_convertrawtemperature(tv >> 8);
}

bool dwhw_init(void)
{
	int ret;

#if DRIVER_VERSION_HEX >= 0x080202
	LOG_INF("Decadriver source: " DRIVER_VERSION_STR);
#elif defined(DRIVER_VERSION_HEX)
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

#if DRIVER_VERSION_HEX >= 0x080202
	ret = dwt_initialise(DWT_READ_OTP_PID | DWT_READ_OTP_LID | DWT_READ_OTP_BAT
						 | DWT_READ_OTP_TMP);
#else
	ret = dwt_initialise(DWT_DW_INIT)
#endif
	if (ret == DWT_ERROR) {
		LOG_ERR("INIT failed");
		return false;
	}

	int cnt = 1000;
	while (!dwt_checkidlerc() && cnt-- > 0) {
		deca_sleep(1);
	}
	if (cnt <= 0) {
		LOG_ERR("Init did not leave IDLE state");
		return false;
	}

	uint32_t dev_id = dwt_readdevid();
	ret = dwt_check_dev_id();
	if (ret != DWT_SUCCESS) {
		LOG_ERR("UNKNOWN DEV ID: %" PRIX32, dev_id);
		return false;
	} else {
		LOG_INF("DEV ID: %" PRIX32, dev_id);
	}

	// dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

	dw3000_spi_speed_fast();

	// assume dwphy_init() is run soon!
	dwhw_update_calib_temp();

	dwchip_ready = true;
	return true;
}

void dwhw_configure_sleep(void)
{
	/* Sleep configuration:
	 * - run RX calibration (PGFCAL)
	 * - restore config
	 * - wakeup on CS pin
	 * - wakeup on WAKEUP
	 * - enable deep sleep */
	dwt_configuresleep(DWT_PGFCAL | DWT_CONFIG,
					   DWT_SLP_EN | DWT_WAKE_CSN | DWT_WAKE_WUP);
}

void dwhw_enable_tx_interrupt(bool on)
{
	dwt_setinterrupt(
#ifdef DRIVER_VERSION_HEX // >= 0x060007
		DWT_INT_TXFRS_BIT_MASK,
#else
		DWT_INT_TFRS,
#endif
		0, on ? DWT_ENABLE_INT : DWT_DISABLE_INT);
}

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

#if DRIVER_VERSION_HEX >= 0x080202
	dwt_restoreconfig(1);
#else
	dwt_restoreconfig();
#endif

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

#if DWHW_DEBUG_WAKEUP
	LOG_INF("WAKEUP STATE %lx STATUS %lx ENABLE %lx",
			dwt_read32bitreg(SYS_STATE_LO_ID), dwt_read32bitreg(SYS_STATUS_ID),
			dwt_read32bitreg(SYS_ENABLE_LO_ID));
#endif

	dwhw_update_calib_temp(); // DWT_PGFCAL was set
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

void dwhw_calib_if_temp_change(void)
{
	uint16_t tv = dwt_readtempvbat();
	float temp = dwt_convertrawtemperature(tv >> 8);
	LOG_INF("TEMP %.2f last %.2f", (double)temp, (double)last_calib_temp);
	if (temp >= last_calib_temp + 20.0f || temp <= last_calib_temp - 20.0f) {
		LOG_WARN("Temperature changed by 20deg, calibrating");
		last_calib_temp = temp;
		dwt_pgf_cal(1);
	}
}
