/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#ifndef DECA_HW_H
#define DECA_HW_H

#include <stdbool.h>

bool dwhw_init(void);
void dwhw_sleep(void);
bool dwhw_wakeup(void);
bool dwhw_is_ready(void);
void dwhw_sleep_after_tx(void);
void dwhw_configure_sleep(void);
void dwhw_enable_tx_interrupt(bool on);

#endif
