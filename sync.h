#ifndef SYNC_H
#define SYNC_H

#include <stdbool.h>
#include <stdint.h>

#include "dwmac.h"

#define SYNC_MSG 0x10

typedef void (*sync_cb_t)(uint16_t src, uint8_t seq, uint64_t tx_ts,
						  uint64_t rx_ts, float skew);

bool sync_send(void);
void sync_handle_msg(const struct rxbuf* rx);
void sync_set_handler(sync_cb_t cb);

#endif
