#pragma once

enum dwevent_e
{
    DWEVT_RX,
    DWEVT_TX_DONE,
};

int dwtask_init();
int dwtask_queue_event(enum dwevent_e type, void* data);
