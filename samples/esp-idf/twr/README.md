# TWR example for ESP-IDF

This is a simple example for two way ranging (TWR) between two ESP32 boards with a DWM3000. This example is configured for the Makerfabs UWB3000 board.

We require the following libraries in the componets directory:
 * https://github.com/br101/dw3000-decadriver-source
 * https://github.com/br101/libdeca

One side should set TWR_RECEIVER to 1 in main.c and will be the receiver.

The other side sets TWR_RECEIVER to 0 and will initate the TWR exchange.
