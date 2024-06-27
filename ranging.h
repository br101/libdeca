#ifndef DECA_RANGING
#define DECA_RANGING

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dwmac.h"

#define TWR_FAILED_VALUE UINT16_MAX
#define TWR_OK_VALUE	 (UINT16_MAX - 1)
#define TWR_MSG_GROUP	 0x20

typedef void (*twr_cb_t)(uint64_t src, uint64_t dst, uint16_t dist,
						 uint16_t num);

bool twr_start(uint16_t dst, bool ss);
void twr_handle_message_short(const struct rxbuf* rx);
void twr_handle_timeout(uint32_t status);
bool twr_in_progress(void);
void twr_cancel(void);
void twr_init(uint8_t rate_dw, uint8_t plen_dw, uint8_t prf_dw);
void twr_set_observer(twr_cb_t cb);

double twr_distance_calculation_dtu(uint32_t poll_rx_ts, uint32_t resp_tx_ts,
									uint32_t final_rx_ts, uint32_t Ra,
									uint32_t Da);

#endif
