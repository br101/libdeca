#include "mac802154.h"

void mac154_set_frame_pending(uint8_t* buf)
{
	buf[0] |= MAC154_FC_FRAME_PEND;
}
