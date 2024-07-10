/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#include <string.h> // memcpy

#include "dwtime.h"
#include "log.h"

static uint64_t dw_last_ts;

uint64_t dw_timestamp_u64(uint8_t ts_tab[5])
{
	uint64_t ts = 0;
	for (int i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

uint64_t dw_get_tx_timestamp(void)
{
	uint8_t ts_tab[5];
	dwt_readtxtimestamp(ts_tab);
	return dw_timestamp_u64(ts_tab);
}

uint64_t dw_get_rx_timestamp(void)
{
	uint8_t ts_tab[5];
	dwt_readrxtimestamp(ts_tab);
	return dw_timestamp_u64(ts_tab);
}

uint64_t dw_get_systime(void)
{
	uint8_t ts_tab[5] = {0};
	dwt_readsystime(&ts_tab[1]);
	return dw_timestamp_u64(ts_tab);
}

void dw_set_buf_timestamp(uint8_t* ts_field, uint64_t ts)
{
	memcpy(ts_field, &ts, 5);
}

uint64_t dw_get_buf_timestamp(const uint8_t* ts_field)
{
	uint64_t ts = 0;
	memcpy(&ts, ts_field, 5);
	return ts;
}

uint64_t dw_timestamp_extend(uint64_t ts)
{
	while (ts < dw_last_ts) {
		ts += (DTU_MASK + 1);
	}
	dw_last_ts = ts;
	return ts;
}
