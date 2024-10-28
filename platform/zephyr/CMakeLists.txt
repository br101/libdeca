# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
    log.c
    dwmac_task.c
    ../../blink.c
    ../../dwhw.c
    ../../dwmac_irq.c
    ../../dwmac.c
    ../../dwphy.c
    ../../dwproto.c
    ../../dwtime.c
    ../../dwutil.c
)

zephyr_include_directories(../..)

target_include_directories(${ZEPHYR_CURRENT_LIBRARY} PRIVATE .)
