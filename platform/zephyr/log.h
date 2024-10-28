#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(deca);

#define LOG_WARN LOG_WRN

#define DBG_UWB(...) LOG_DBG(__VA_ARGS__)

#define LOG_INF_IRQ(...) LOG_INF(__VA_ARGS__)
#define LOG_ERR_IRQ(...) LOG_ERR(__VA_ARGS__)

#if CONFIG_DECA_DEBUG_OUTPUT_IRQ
#define DBG_UWB_IRQ(...) LOG_INF(__VA_ARGS__)
#else
#define DBG_UWB_IRQ(...) // don't log
#endif

#define LOG_HEXDUMP(_txt, _buf, _len) LOG_HEXDUMP_INF(_buf, _len, _txt)
