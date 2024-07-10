/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#pragma once

enum dwevent_e
{
    DWEVT_RX,
    DWEVT_RX_TIMEOUT,
    DWEVT_TX_DONE,
    DWEVT_ERR,
};

int dwtask_init();
int dwtask_queue_event(enum dwevent_e type, const void* data);
