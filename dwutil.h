#ifndef DECA_UTIL_H
#define DECA_UTIL_H

#define ADDR_FMT  "%04X"

#if ESP_PLATFORM
#define LADDR_FMT "[%016llX]"
#define LADDR_PAR(x) x
#else
// 64 bit printf is not available in NRF SDK
#define LADDR_FMT "[%08lX%08lX]"
#define LADDR_PAR(x) (uint32_t)(x >> 32), (uint32_t)x
#endif

#define ASSERT_RET(cond)                                                       \
	if (!(cond)) {                                                             \
		LOG_ERR("ASSERT " #cond " in %s:%d", __FILE__, __LINE__);              \
		return false;                                                          \
	}

#ifndef CEIL_DIV
#define CEIL_DIV(A, B) (((A) + (B) - 1) / (B))
#endif

#endif
