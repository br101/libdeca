#include <string.h> // memcpy

#include "dwtime.h"

uint64_t deca_timestamp_u64(uint8_t ts_tab[5])
{
	uint64_t ts = 0;
	for (int i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

uint64_t deca_get_tx_timestamp(void)
{
	uint8_t ts_tab[5];
	dwt_readtxtimestamp(ts_tab);
	return deca_timestamp_u64(ts_tab);
}

uint64_t deca_get_rx_timestamp(void)
{
	uint8_t ts_tab[5];
	dwt_readrxtimestamp(ts_tab);
	return deca_timestamp_u64(ts_tab);
}

/** note SYS time is latched after read and only updated after a SPI write */
uint64_t deca_get_sys_time(void)
{
	uint8_t ts_tab[5] = {0};
	dwt_readsystime(&ts_tab[1]);
	return deca_timestamp_u64(ts_tab);
}

void deca_set_buf_timestamp(uint8_t* ts_field, uint64_t ts)
{
	memcpy(ts_field, &ts, 5);
}

uint64_t deca_get_buf_timestamp(const uint8_t* ts_field)
{
	uint64_t ts = 0;
	memcpy(&ts, ts_field, 5);
	return ts;
}
