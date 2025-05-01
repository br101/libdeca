#include "dwtime.h"
#include "log.h"
#include <deca_device_api.h>
#include <zephyr/timing/timing.h>

static int sizes[] = {10, 12, 14, 15, 16, 18, 20, 50, 100, 200, 512};
static unsigned char buf[1024] = {1};

#define DWTEST_REPETITIONS 1000

#if defined(__ZEPHYR__) && CONFIG_TIMING_FUNCTIONS

void dwtest_spi(void)
{
	timing_t start_time, end_time;
	uint64_t total_cycles;
	uint64_t total_ns;

	timing_init();
	timing_start();

	for (size_t i = 0; i < ARRAY_SIZE(sizes); i++) {
		start_time = timing_counter_get();
		for (int j = 0; j < DWTEST_REPETITIONS; j++) {
			dwt_writetxdata(sizes[i], buf, 0);
		}
		end_time = timing_counter_get();
		total_cycles = timing_cycles_get(&start_time, &end_time);
		total_ns = timing_cycles_to_ns_avg(total_cycles, DWTEST_REPETITIONS);
		LOG_INF("SPI %d %f usec /byte %f", sizes[i], total_ns / 1000.0,
				total_ns / sizes[i] / 1000.0);
	}
	timing_stop();
}

#else

void dwtest_spi(void)
{
	uint64_t start;
	uint64_t end;
	uint32_t diff;

	for (size_t i = 0; i < ARRAY_SIZE(sizes); i++) {
		start = dw_get_systime();
		for (int j = 0; j < 1000; j++) {
			dwt_writetxdata(sizes[i], buf, 0);
		}
		end = dw_get_systime();
		diff = (end - start) / 1000;
		LOG_INF("SPI %d %f usec /byte %f", sizes[i], DTU_TO_US(diff),
				DTU_TO_US(diff / sizes[i]));
	}
}
#endif
