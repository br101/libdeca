#include <inttypes.h>
#include <stdio.h>

#include "dw3000_hw.h"
#include "dwhw.h"
#include "dwmac.h"
#include "dwphy.h"
#include "dwproto.h"
#include "ranging.h"

#include "log.h"

// Set this on one side to receive and reply to TWR messages
// Otherwise we will initate TWR
#define TWR_RECEIVER 0

#define PANID 0xdeca
#if TWR_RECEIVER
#define MAC16 0x0001
#else
#define MAC16 0x0002
#endif

static const char* LOG_TAG = "TWR_TEST";

#if !TWR_RECEIVER
static void twr_done_cb(uint64_t src, uint64_t dst, uint16_t dist, uint16_t num)
{
	LOG_INF("TWR Done %04X: %d cm", (uint16_t)dst, dist);
}
#endif

void app_main(void)
{
	LOG_INF("libdeca TWR test");

	// decadriver init
	dw3000_hw_init();
	dw3000_hw_reset();
	dw3000_hw_init_interrupt();

	// libdeca init
	dwhw_init();
	dwphy_config();
	dwphy_set_antenna_delay(DWPHY_ANTENNA_DELAY);
	dwmac_init(PANID, MAC16, dwprot_rx_handler, NULL, NULL);
	dwmac_set_frame_filter();

	// TWR
	twr_init(2000, true);

#if TWR_RECEIVER

	// Just enable receive mode, dwprot_rx_handler will handle TWR
	LOG_INF("Receiver start");
	dwmac_set_rx_reenable(true);
	dwt_forcetrxoff();
	dwt_rxenable(DWT_START_RX_IMMEDIATE);

#else

	// Initiate two way ranging to 0x0001
	LOG_INF("TWR to 0x0001");
	twr_set_observer(twr_done_cb);
	twr_start(0x0001);

#endif
}
