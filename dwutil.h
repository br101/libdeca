/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#ifndef DECA_UTIL_H
#define DECA_UTIL_H

#include <stdint.h>

#define ADDR_FMT "%04X"

#if ESP_PLATFORM
#define LADDR_FMT	 "[%016llX]"
#define LADDR_PAR(x) x
#else
// 64 bit printf is not available in NRF SDK
#define LADDR_FMT	 "[%08lX%08lX]"
#define LADDR_PAR(x) (uint32_t)(x >> 32), (uint32_t)x
#endif

#define IS_SHORT_ADDR(x) (((x) & 0xffffffffffff0000LL) == 0x0000000000000000LL)

#define ASSERT_RET(cond)                                                       \
	if (!(cond)) {                                                             \
		LOG_ERR("ASSERT " #cond " in %s:%d", __FILE__, __LINE__);              \
		return false;                                                          \
	}

#ifndef CEIL_DIV
#define CEIL_DIV(A, B) (((A) + (B) - 1) / (B))
#endif

uint64_t mac_to_eui64(const uint8_t* mac);

#endif
