#ifndef BLINK_H
#define BLINK_H

#include <stdint.h>
#include <stdbool.h>

#include "dwmac.h"

#define BLINK_MSG 0x11

bool blink_send(void);
void blink_handle_msg(const struct rxbuf* rx);

#endif
