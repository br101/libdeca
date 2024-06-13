# libdeca - Library for Qorvo/Decawave DW3000

This is a library with convenient functions and abstractions on top of the API provided by `decadriver` and `dwt_uwb_driver`.

It is tailored to be used with [dw3000-decadriver-source](https://github.com/br101/dw3000-decadriver-source), but it can also be used with the binary-only library `dwt_uwb_driver` from Qorvo.

It provides:
 * Simplified initialization functions
 * Deferred IRQ handling for the NRF-SDK and ESP-IDF platforms
 * Functions for getting recommended PHY parameters
 * A convenint API for defining TX buffer properties
 * Helpers for converting time units
 * A simple implementation of two-way ranging (TWR)
 * Some definitions for IEEE 802.15.4 frame formats

Most of the code is pure platform-independent C code, and can be used in anywhere, but IRQ handling is platform specific and implemented for:

 * ESP-IDF (version 5.1.1)
 * NRF SDK (version 17.1.0)

## ESP-IDF

For ESP-IDF it can be used by adding it to the components directory.

## NRF-SDK v17.1.0

Just add the necessary files to your Makefile or IDE. Define the log functions in log.h

## Usage for TWR

Here is an example for initializing the library and using it for TWR:

```
static struct twr_res res[2];

static void twr_done_cb(int num)
{
	if (num == 4) {
		LOG_INF("TWR Done %04X: %d cm", res[0].addr, res[0].dist);
	} else {
		LOG_ERR("Unexpected TWR result %d", num);
	}
}

static void twr_rx_handler(const struct rxbuf* rx)
{
	if ((rx->u.s.func & 0xF0) == 0x20) {
		twr_handle_message(rx);
	}
}

void test_twr(void)
{
  // decadriver init
  dw3000_hw_init(&dw_hw_cfg);
  dw3000_hw_reset();
  dw3000_hw_init_interrupt();

  // libdeca init
  dwhw_init();
  dwphy_config();
  dwphy_set_antenna_delay(DWPHY_ANTENNA_DELAY);
  dwmac_init(PANID, MAC16, 10, twr_rx_handler, your_timeout_handler,
  			     your_error_handler);
  dwmac_set_frame_filter();
  twr_init(DWT_BR_6M8, DWT_PLEN_64, DWT_PRF_64M);

  // two way ranging to two destinations
  uint16_t dst[] = {0x0002, 0x0003};
  twr_start(dst, 2, 1, res, sizeof(res), twr_done_cb, false);
}
```

The other side is initialized the same, but instead of `twr_start()` has to enable receive mode:

```
dwmac_set_rx_reenable(true);
dwt_forcetrxoff();
dwt_rxenable(DWT_START_RX_IMMEDIATE);
```

If you get error messages like
```
E (4983) DECA: TX error seq 0 (0x4080e1a0)
E (4983) DECA:  SYS Time:       442f907a00
E (4993) DECA:  TX Time:        442ea48234
E (4993) DECA:  Diff:           ff140834 (-242 us)
```
the interrupt processing on your CPU is not fast enough (in this case we wanted to transmit a packet at a certain time but we were 242us too late). You can increase `TWR_PROCESSING_TIME` in `ranging.c`, but they have to be the same on **both sides** (transmit and receive). For more exact measurements it's better to have a lower number here. Also note that logging, especially in interrupt context in `dwmac_irq.c` can have an impact on the processing time, so after you are sure you get the right interrupts, it's better to disable logging there.
