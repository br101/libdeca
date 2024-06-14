#ifndef BLINK_H
#define BLINK_H

#include <stdbool.h>
#include <stdint.h>

#include "dwmac.h"

#define BLINK_MSG 0x11

typedef void (*blink_cb_t)(uint16_t src, uint8_t seq, uint64_t rx_ts);

bool blink_send(void);
void blink_handle_msg(const struct rxbuf* rx);
void blink_set_handler(blink_cb_t cb);

#endif
