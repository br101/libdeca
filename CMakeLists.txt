# This is for using libdeca as a component in ESP-IDF
if(CONFIG_DW3000)
idf_component_register(SRCS dwhw.c dwmac.c dwmac_irq.c dwphy.c dwtime.c ranging.c
                            platform/esp-idf/dwmac_task.c blink.c sync.c dwproto.c
                            mac802154.c dwutil.c dwtest.c
                       INCLUDE_DIRS "."
                       PRIV_INCLUDE_DIRS platform platform/esp-idf/priv
                       PRIV_REQUIRES "decadriver")
endif()
