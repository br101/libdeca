#ifndef DECA_UTIL_H
#define DECA_UTIL_H

#define ADDR_FMT "%04X"

#define ASSERT_RET(cond)                                                       \
	if (!(cond)) {                                                             \
		LOG_ERR("ASSERT " #cond " in %s:%d", __FILE__, __LINE__);              \
		return false;                                                          \
	}

#ifndef CEIL_DIV
#define CEIL_DIV(A, B)      \
    (((A) + (B) - 1) / (B))
#endif

#endif
