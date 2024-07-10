/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#include "mac802154.h"

void mac154_set_frame_pending(uint8_t* buf)
{
	buf[0] |= MAC154_FC_FRAME_PEND;
}
