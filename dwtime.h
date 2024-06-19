#ifndef DWTIME_H
#define DWTIME_H

#include <deca_device_api.h>
#include <stdint.h>

/* DTU (DWT_TIME_UNITS) is 15.65 picoseconds ticks (1.0/499.2e6/128.0
 * = 15.65e-12) */
#define DTU_TO_PS(x) ((x) * DWT_TIME_UNITS)
#define DTU_TO_NS(x) ((x) * DWT_TIME_UNITS * 1000000000.0)
#undef DTU_TO_US
#undef US_TO_DTU
#define DTU_TO_US(x) ((x) * DWT_TIME_UNITS * 1000000.0)
#define DTU_TO_MS(x) ((x) * DWT_TIME_UNITS * 1000.0)
#define US_TO_DTU(x) ((x) / DWT_TIME_UNITS / 1000000.0)
#define MS_TO_DTU(x) ((x) / DWT_TIME_UNITS / 1000.0)

/* UWB microsecond (UUS) to device time unit (DTU)
 * 1 UUS = 512 / 499.2 µs and 1 µs = 499.2 * 128 DTU */
#define UUS_TO_DTU(x) ((x) * 65536)
#define UUS_TO_US(x)  ((x) * 512 / 499.2)
#define US_TO_UUS(x)  ((x) * 499.2 / 512.0)

/* Delayed TRX timestamp is 8 ns resolution - mask lower 9 bits and shift 8 */
#define DTU_DELAYEDTRX_MASK	 0xFFFFFFFE00L
#define DTU_TO_DELAYEDTRX(x) (((x) & DTU_DELAYEDTRX_MASK) >> 8)

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

#define TIME_TO_DISTANCE(x) ((x) * SPEED_OF_LIGHT)
#define DTU_TO_DISTANCE(x)	(DTU_TO_PS(x) * SPEED_OF_LIGHT)

#define DTU_MASK 0xFFFFFFFFFFL

#define TS_FMT	"%016llx"
#define DWT_FMT "%02x%02x%02x%02x%02x"
#define DWT_PAR(x)                                                             \
	(uint8_t)(x >> 32), (uint8_t)(x >> 24), (uint8_t)(x >> 16),                \
		(uint8_t)(x >> 8), (uint8_t)(x)
#define DWT_PARA(x) x[4], x[3], x[2], x[1], x[0]

#define LOG_DBG_TS(txt, t) LOG_DBG(txt DWT_FMT, DWT_PAR((t)))
#define LOG_INF_TS(txt, t) LOG_INF(txt DWT_FMT, DWT_PAR((t)))
#define LOG_ERR_TS(txt, t) LOG_ERR(txt DWT_FMT, DWT_PAR((t)))

/** convert 5 byte timestamp to uint64_t */
uint64_t dw_timestamp_u64(uint8_t ts_tab[5]);
uint64_t dw_get_tx_timestamp(void);
uint64_t dw_get_rx_timestamp(void);
/** note SYS time is latched after read and only updated after a SPI write */
uint64_t dw_get_systime(void);
void dw_set_buf_timestamp(uint8_t* ts_field, uint64_t ts);
uint64_t dw_get_buf_timestamp(const uint8_t* ts_field);

#endif
