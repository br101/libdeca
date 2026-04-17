# TWR example for ESP-IDF

This is a simple example for two way ranging (TWR) between two ESP32 boards with a DWM3000. This example is configured for the Makerfabs UWB3000 board and has been tested with ESP-IDF 5.5.3.

One side should set TWR_RECEIVER to 1 in main.c and will be the receiver.

The other side sets TWR_RECEIVER to 0 and will initate the TWR exchange.


## Dependencies

We require the following libraries in the components directory:
 * https://github.com/br101/dw3000-decadriver-source
 * https://github.com/br101/libdeca

  cd components
  git clone https://github.com/br101/dw3000-decadriver-source decadriver
  git clone https://github.com/br101/libdeca


## Building

  idf.py -p /dev/ttyUSB1 -D SDKCONFIG_DEFAULTS="sdkconfig.makerfabs_uwb3000" build flash monitor
