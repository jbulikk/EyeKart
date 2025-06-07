#include "bt_peripheral.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/printk.h>
#include <string.h>

static char value_buf[100] = "Init";
static bt_peripheral_recv_cb_t recv_cb = NULL;

static ssize_t read_func(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len, uint16_t offset)
{
    const char *value = attr->user_data;
    size_t value_len = strlen(value);

    if (offset > value_len) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    size_t to_copy = MIN(len, value_len - offset);
    memcpy(buf, value + offset, to_copy);
    return to_copy;
}

static ssize_t write_func(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (offset + len >= sizeof(value_buf)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value_buf + offset, buf, len);
    value_buf[offset + len] = '\0';

    if (recv_cb) {
        recv_cb(value_buf);
    }

    return len;
}

// GATT service definition using standard Zephyr macros (no internal symbols)
BT_GATT_SERVICE_DEFINE(my_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(
        0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0))),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(
        0xabcdefab, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        read_func, write_func, value_buf),
);

int bt_peripheral_init(bt_peripheral_recv_cb_t cb)
{
    int err;
    struct bt_le_adv_param adv_params = {
        .options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_CODED,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,  // 11.25 ms * 2 = 22.5 ms
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,  // 11.25 ms * 4 = 45 ms
        .id = BT_ID_DEFAULT,
    };

    recv_cb = cb;

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return err;
    }

    err = bt_le_adv_start(&adv_params, NULL, 0, NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return err;
    }

    printk("Peripheral advertising started with Coded PHY\n");
    return 0;
}