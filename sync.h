#ifndef SYNC_H
#define SYNC_H

#include <stdint.h>
#include <stdbool.h>

#include "dwmac.h"

#define SYNC_MSG 0x10

bool sync_send(void);
void sync_handle_msg(const struct rxbuf* rx);

#endif
