#ifndef BT_CENTRAL_H
#define BT_CENTRAL_H

typedef void (*bt_central_connected_cb_t)(void);
int bt_central_init(bt_central_connected_cb_t cb);
int bt_central_send(const char *data);

#endif // BT_CENTRAL_H