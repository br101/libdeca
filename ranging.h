#ifndef DECA_RANGING
#define DECA_RANGING

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dwmac.h"

#define TWR_MAX_DST		 10
#define TWR_FAILED_VALUE UINT16_MAX
#define TWR_OK_VALUE	 (UINT16_MAX - 1)

struct twr_res {
	uint16_t addr;
	uint16_t dist;
};

bool twr_start(const uint16_t dst[], size_t len, uint16_t cnum,
			   void* res, size_t res_len, void (*done_handler)(int res),
			   bool ss);
void twr_handle_message(struct rxbuf* rx);
void twr_handle_timeout(uint32_t status);
bool twr_in_progress(void);
void twr_cancel(void);
void twr_init(uint8_t rate_dw, uint8_t plen_dw, uint8_t prf_dw);

double twr_distance_calculation_dtu(uint32_t poll_rx_ts, uint32_t resp_tx_ts,
									uint32_t final_rx_ts, uint32_t Ra,
									uint32_t Da);

#endif
