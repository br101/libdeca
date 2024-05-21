# libdeca - Library for using Qorvo/Decawave DW3000

This is a library with convenient functions and abstractions on top of the API provided by `decadriver` and `dwt_uwb_driver`.

It is tailored to be used with [dw3000-decadriver-source](https://github.com/br101/dw3000-decadriver-source), but it can also be used with the later binary-only library `dwt_uwb_driver` from Qorvo.

It provides:
 * Practical initialization functions
 * Deferred IRQ handling for the NRF-SDK and ESP-IDF platforms
 * Functions for getting recommended PHY parameters
 * A convenint API for defining TX buffer properties
 * Helpers for converting time units
 * A simple implementation of two-way ranging (TWR)
 * Some definitions for IEEE 802.15.4 frame formats

## Supported platforms

Most of the code is pure platform-independent C code, and can be used in anywhere, but IRQ handling is platform specific and implemented for
 * NRF-SDK (v17.1.0)
 * ESP-IDF (v5.1.1)
