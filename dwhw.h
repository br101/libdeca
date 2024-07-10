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
#include <stddef.h>
#include <stdint.h>

bool dwhw_init(void);
void dwhw_sleep(void);
bool dwhw_wakeup(void);
bool dwhw_is_ready(void);

uint8_t dwhw_read_battery(void);
const char* dwhw_batt_str(uint8_t raw);

void dwhw_fmt_temp_volt(uint16_t tempbat, char* temps, char* bats, int len);
float dwhw_conv_temp(uint8_t temp, char* temps, size_t len);

#endif
