#ifndef DECA_RANGING
#define DECA_RANGING

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dwmac.h"

/** TWR_PROCESSING_DELAY: the processing delay may need to be increased for
 * different processor and IRQ handling speeds */
#define TWR_PROCESSING_DELAY 600 /* us */
#define TWR_FAILED_VALUE	 UINT16_MAX
#define TWR_OK_VALUE		 (UINT16_MAX - 1)
#define TWR_MSG_GROUP		 0x20

typedef void (*twr_cb_t)(uint64_t src, uint64_t dst, uint16_t dist,
						 uint16_t num);

/** Initialize TWR for a specific phy setting and delay */
void twr_init(uint8_t rate_dw, uint8_t plen_dw, uint8_t prf_dw,
			  uint32_t processing_delay_us);
/** Start DS-TWR sequence to ancor */
bool twr_start(uint64_t dst);
/** Start SS-TWR sequence to ancor */
bool twr_start_ss(uint64_t dst);
bool twr_in_progress(void);
void twr_cancel(void);
void twr_set_observer(twr_cb_t cb);

void twr_handle_message(const struct rxbuf* rx);
double twr_distance_calculation_dtu(uint32_t poll_rx_ts, uint32_t resp_tx_ts,
									uint32_t final_rx_ts, uint32_t Ra,
									uint32_t Da);

#endif
