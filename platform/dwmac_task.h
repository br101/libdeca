#pragma once

enum dwevent_e
{
    DWEVT_RX,
    DWEVT_RX_TIMEOUT,
    DWEVT_TX_DONE,
    DWEVT_ERR,
};

int dwtask_init();
int dwtask_queue_event(enum dwevent_e type, const void* data);
