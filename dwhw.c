
#include <string.h>
#include <stdio.h>

#include <deca_version.h>
#include <deca_device_api.h>
#if DW3000_DRIVER_VERSION >= 0x060007
#include <deca_probe_interface.h>
#endif
#include <dw3000_hw.h>
#include <dw3000_spi.h>

#include "dwhw.h"
#include "dwphy.h"

#include "log.h"

static const char* LOG_TAG = "DECA";

static bool dwchip_ready = false;

bool dwhw_init(void)
{
	int ret;

#if DW3000_DRIVER_VERSION >= 0x060007
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
	};
	if (cnt <= 0) {
		LOG_ERR("did not leave IDLE state");
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
	 * - enable sleep mode */
	dwt_configuresleep(0, DWT_SLP_EN | DWT_WAKE_CSN);
	dwt_entersleep(DWT_DW_INIT);
}

bool dwhw_wakeup(void)
{
	if (dwchip_ready) {
		LOG_ERR("Already woke up?");
		return false;
	}

	LOG_INF("Wakeup");
	dw3000_hw_wakeup();

	/*
	 * After asserting the RESET pin, the chip in in INIT state and we need
	 * to wait until CLKPLL locked and IDLE state has been reached.
	 *
	 * Also the DW1000 User Manual writes: "Care should be taken not to have
	 * an active SPI access in progress at the CLKPLL lock time (i.e. at
	 * t = 5 μs) when the automatic switch from the INIT state to the IDLE
	 * state is occurring, because the switch-over of clock source can cause
	 * bit errors in the SPI transactions.
	 *
	 * Or from DWM1001-Dev Simple Examples "it takes ~35us in total for the
	 * DW1000 to lock the PLL, download AON and go to IDLE state"
	 */
	deca_sleep(1);

	int ret = dwt_check_dev_id();
	if (ret != DWT_SUCCESS) {
		LOG_ERR("Failed to read device id after wakeup!");
		return false;
	}

#if DW3000_DRIVER_VERSION >= 0x060007
	dwt_softreset(0);
#else
	dwt_softreset();
#endif

	dwchip_ready = true;
	return true;
}

const char* dwhw_batt_str(uint8_t raw)
{
	static char bats[6];
	float bf = 0.0057 * raw + 2.3;

#if NO_FLOAT_PRINTF
	int r = double_to_str(bf, bats, sizeof(bats));
#else
	int r = snprintf(bats, sizeof(bats), "%.2f", bf);
#endif
	if (r > 0 && (size_t)r >= sizeof(bats)) {
		bats[sizeof(bats) - 1] = '\0';
	}
	return bats;
}

float dwhw_conv_temp(uint8_t temp, char* temps, size_t len)
{
	float tf = 1.13 * temp - 113.0;

	if (temps != NULL && len > 1) {
#if NO_FLOAT_PRINTF
		int r = double_to_str(tf, temps, len);
#else
		int r = snprintf(temps, len, "%.2f", tf);
#endif
		if (r > 0 && (size_t)r >= len) {
			temps[len - 1] = '\0';
		}
	}
	return tf;
}

void dwhw_fmt_temp_volt(uint16_t tempbat, char* temps, char* bats, int len)
{
	uint8_t temp = tempbat >> 8;
	uint8_t bat = (uint8_t)tempbat;

	dwhw_conv_temp(temp, temps, len);
	strncpy(bats, dwhw_batt_str(bat), len);
}

uint8_t dwhw_read_battery(void)
{
	if (!dwhw_is_ready()) {
		return 0xff;
	}

	return (uint8_t)dwt_readtempvbat(); // battery is lower byte
}

bool dwhw_is_ready(void)
{
	if (!dwchip_ready) {
		LOG_ERR("***NOT READY!***");
		return false;
	}
	return true;
}
