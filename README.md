# libdeca - UWB Library for Qorvo/Decawave DW3000

This is a library with convenient functions and abstractions on top of the API provided by `decadriver` and `dwt_uwb_driver`.

It is tailored to be used with [dw3000-decadriver-source](https://github.com/br101/dw3000-decadriver-source), but it can also be used with the binary-only library `dwt_uwb_driver` from Qorvo.

It provides:
 * Simplified initialization functions
 * Deferred IRQ handling for the NRF-SDK and ESP-IDF platforms
 * Functions for getting recommended PHY parameters
 * A convenint API for defining TX buffer properties
 * Helpers for converting time units
 * A simple to use implementation of two-way ranging (TWR)
 * Some definitions for IEEE 802.15.4 frame formats
 * Blink and Sync messages

Most of the code is pure platform-independent C code, and can be used anywhere, but IRQ handling is platform specific and implemented for:

 * Zephyr (version 3.6, NRF Connect SDK v2.7.0)
 * ESP-IDF (version 5.1.1)
 * NRF SDK (version 17.1.0)

## Zephyr

Add this to the main CMakeLists.txt:
```
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdeca/platform)
```

## ESP-IDF

For ESP-IDF it can be used by adding it to the components directory.

## NRF-SDK v17.1.0

Just add the necessary files to your Makefile or IDE. Define the log functions in log.h

## Usage for TWR

Here is an example for initializing the library and using it for TWR:

```
#include "dw3000_hw.h"
#include "dwhw.h"
#include "dwmac.h"
#include "dwphy.h"
#include "dwproto.h"

static void twr_done_cb(uint64_t src, uint64_t dst, uint16_t dist,
						 uint16_t num)
{
  LOG_INF("TWR Done %04X: %d cm", (uint16_t)dst, dist);
}

void test_twr(void)
{
  // decadriver init
  dw3000_hw_init();
  dw3000_hw_reset();

  // libdeca init
  dwhw_init();
  dwphy_config();
  dwphy_set_antenna_delay(DWPHY_ANTENNA_DELAY);
  dwmac_init(PANID, MAC16, dwprot_rx_handler, your_timeout_handler, your_error_handler);
  dwmac_set_frame_filter();
  twr_init(TWR_PROCESSING_DELAY);
  twr_set_observer(twr_done_cb);
  // two way ranging to 0x0001
  twr_start(0x0001);
}
```

The other side is initialized the same, but instead of `twr_start()` has to enable receive mode:

```
dwmac_set_rx_reenable(true);
dwt_forcetrxoff();
dwt_rxenable(DWT_START_RX_IMMEDIATE);
```

If you get error messages like this:
```
E (4983) DECA: TX error seq 0 (0x4080e1a0)
E (4983) DECA:  SYS Time:       442f907a00
E (4993) DECA:  TX Time:        442ea48234
E (4993) DECA:  Diff:           ff140834 (-242 us)
```
then the interrupt processing on your CPU is not fast enough (in this case we wanted to transmit a packet at a certain time but we were 242us too late). You can increase `TWR_PROCESSING_TIME` in `ranging.h`, or pass a different number to twr_init() but they **have to be the same on both sides** (transmit and receive). For more exact distance measurements it's better to have a lower number here. Also note that logging, especially in interrupt context in `dwmac_irq.c` can have an impact on the processing time, so after you are sure you get the right interrupts, it's better to disable logging there.


## License ##

Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)

This library is licensed under the GNU Lesser General Public License,
Version 3. See the file LICENSE.txt for more details.
