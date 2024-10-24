/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#ifndef DECA_PHY_H
#define DECA_PHY_H

#include <stdbool.h>
#include <stdint.h>

#include "dwutil.h"

/* Default antenna delay values for 64 MHz PRF, usually calibrated
 * (PRF16: 16418) */
// #define DWPHY_ANTENNA_DELAY 16436
// #define DWPHY_ANTENNA_DELAY 16496
// #define DWPHY_ANTENNA_DELAY 16384
// #define DWPHY_ANTENNA_DELAY 16150
// #define DWPHY_ANTENNA_DELAY 16405
#define DWPHY_ANTENNA_DELAY 16368

/* the phy_get_[preable/sfd/phyhdr/data/packet]_time functions return in units
 * of (picoseconds / 10) = (nanoseconds * 100), so this macro can be used to
 * convert to microseconds */
#define PKTTIME_TO_USEC(ft) CEIL_DIV(ft, 100000)

bool dwphy_config(void);
void dwphy_set_antenna_delay(uint16_t antdelay);

/* phy time calculations */
const char* dwphy_rate_str(uint8_t br);
int dwphy_rate_int(uint8_t br);
int dwphy_plen_int(uint16_t pl);
int dwphy_prf_int(uint8_t p);
int dwphy_pac_int(uint8_t p);
uint32_t dwphy_calc_preamble_time(uint8_t plen_dwt, uint8_t prf_dwt,
								  uint8_t rate_dwt);
uint32_t dwphy_calc_sfd_time(uint8_t prf_dwt, uint8_t rate_dwt);
uint32_t dwphy_calc_phyhdr_time(uint8_t rate_dwt);
uint64_t dwphy_calc_data_time(uint8_t rate_dwt, int len);
uint64_t dwphy_calc_packet_time(uint8_t rate_dwt, uint8_t plen_dwt,
								uint8_t prf_dwt, int data_len);
int dwphy_usec_to_pac(uint32_t x);
int dwphy_pac_to_usec(uint16_t pacs);
int dwphy_get_recommended_preambletimeout(void);
void dwphy_print_packet_times(void);

/* clock */
float dwphy_get_rx_clock_offset_ci(int32_t ci);
void dwphy_xtal_trim(void);

/* get / set config */
void dwphy_set_rate(uint8_t br);
uint8_t dwphy_get_rate(void);
void dwphy_set_plen(uint8_t plen);
uint8_t dwphy_get_plen(void);
uint8_t dwphy_get_prf(void);

#endif
