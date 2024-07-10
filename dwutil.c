/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#include "dwutil.h"

uint64_t mac_to_eui64(const uint8_t* mac)
{
	uint64_t eui64;
	uint8_t* m64 = (uint8_t*)&eui64;
	m64[0] = mac[0];
	m64[1] = mac[1];
	m64[2] = mac[2];
	// m64[3] = 0xFF;
	// m64[4] = 0xFE;
	m64[3] = mac[3];
	m64[4] = mac[4];
	m64[5] = mac[5];
	m64[6] = 0;
	m64[7] = 0;
	return eui64;
}
