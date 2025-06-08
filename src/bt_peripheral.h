#ifndef BT_PERIPHERAL_H
#define BT_PERIPHERAL_H

typedef void (*bt_peripheral_recv_cb_t)(const char *data);

int bt_peripheral_init(bt_peripheral_recv_cb_t cb);

#endif
