#include "bt_central.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#define TARGET_NAME "Pixel 8 Pro"

static struct bt_conn *default_conn;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_write_params write_params;
static bt_central_connected_cb_t connected_cb;
static struct bt_uuid_128 remote_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0xabcdefab, 0x1234, 0x5678, 0x1234, 0x56789abcdef0));

static uint16_t char_handle;

// Helper to check advertised name
static bool check_name(struct bt_data *data, void *user_data)
{
    const char *name = user_data;

    if (data->type == BT_DATA_NAME_COMPLETE) {
        if (strlen(name) == data->data_len &&
            !memcmp(data->data, name, data->data_len)) {
            return true;
        }
    }
    return false;
}

static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (!attr) {
        printk("Discovery complete\n");
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
    char_handle = chrc->value_handle;
    printk("Discovered char handle: 0x%04x\n", char_handle);

    if (connected_cb) {
        connected_cb();
    }

    return BT_GATT_ITER_STOP;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
        return;
    }

    printk("Connected() callback triggered, err = %d\n", err);
    default_conn = bt_conn_ref(conn);
    discover_params.uuid = &remote_char_uuid.uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = 0x0001;
    discover_params.end_handle = 0xffff;
    discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    bt_gatt_discover(default_conn, &discover_params);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
}

struct device_name_check {
    const char *name;
    bool found;
};

static bool device_found(struct bt_data *data, void *user_data)
{
    struct device_name_check *check = user_data;

    if (data->type == BT_DATA_NAME_COMPLETE) {
        if (strlen(check->name) == data->data_len &&
            !memcmp(data->data, check->name, data->data_len)) {
            check->found = true;
            return false; // stop parsing further (optional)
        }
    }
    return true; // continue parsing
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                    struct net_buf_simple *ad)
{
    if (default_conn) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    struct device_name_check check = {
        .name = TARGET_NAME,
        .found = false,
    };

    bt_data_parse(ad, device_found, &check);

    if (check.found) {
        printk("Found target device: %s (RSSI %d), connecting...\n", addr_str, rssi);

        bt_le_scan_stop();

        struct bt_conn_le_create_param *create_param =
            BT_CONN_LE_CREATE_PARAM(BT_CONN_LE_OPT_NONE,
                                   BT_GAP_SCAN_FAST_INTERVAL,
                                   BT_GAP_SCAN_FAST_INTERVAL);
        struct bt_le_conn_param *conn_param = BT_LE_CONN_PARAM_DEFAULT;

        int err = bt_conn_le_create(addr, create_param, conn_param, &default_conn);
        if (err) {
            printk("Connection failed: %d\n", err);
        }
        // printk("Connected: %d\n", err);
    } else {
        printk("Skipping device: %s (name not matched)\n", addr_str);
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};


int bt_central_init(bt_central_connected_cb_t cb)
{
    int err;
    connected_cb = cb;

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return err;
    }

    bt_conn_cb_register(&conn_callbacks);

    err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);
    if (err) {
        printk("Scanning failed to start (err %d)\n", err);
        return err;
    }

    printk("Central scanning started\n");
    return 0;
}

int bt_central_send(const char *data)
{
    if (!default_conn || !char_handle) {
        printk("Not connected or handle not discovered\n");
        return -1;
    }

    write_params.handle = char_handle;
    write_params.offset = 0;
    write_params.data = data;
    write_params.length = strlen(data);
    write_params.func = NULL;

    // return bt_gatt_write(default_conn, &write_params);
    return bt_gatt_write_without_response(default_conn, char_handle, data, strlen(data), false);

}
