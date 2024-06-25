#ifndef BLINK_H
#define BLINK_H

#include <stdbool.h>
#include <stdint.h>

#include "dwmac.h"

#define BLINK_MSG 0x11

typedef void (*blink_cb_t)(uint64_t src, uint32_t seq, uint64_t rx_ts,
						   uint64_t time_ms, uint8_t battery);

bool blink_send_short(void);
void blink_handle_msg_short(const struct rxbuf* rx);
void blink_set_handler(blink_cb_t cb);

#endif
