#ifndef BLINK_H
#define BLINK_H

#include <stdbool.h>
#include <stdint.h>

#include "dwmac.h"

typedef void (*blink_cb_t)(uint64_t src, uint32_t seq, uint64_t rx_ts,
						   uint64_t time_ms, uint8_t battery);

/* Note: short blink can only be received when frame filtering is disabled */
bool blink_send_short(uint16_t src);
void blink_handle_msg_short(const struct rxbuf* rx);
bool blink_send_long(uint64_t src);
void blink_handle_msg_long(const struct rxbuf* rx);
void blink_set_observer(blink_cb_t cb);

#endif
