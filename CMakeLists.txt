# This is for using libuwifi as a component in ESP-IDF

idf_component_register(SRCS dwhw.c dwmac.c dwphy.c dwtime.c ranging.c
                            platform/esp-idf/dwmac_task.c
                       INCLUDE_DIRS "."
                       PRIV_INCLUDE_DIRS "platform"
                       PRIV_REQUIRES "decadriver")
