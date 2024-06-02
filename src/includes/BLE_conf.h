#ifndef BLE_CONF_H
#define BLE_CONF_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include "i2c_sensors.h"
#include "utils.h"
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include "flash_utils.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/smf.h>

#define CONFIG_SETTINGS 1


#define GATT_PERM_READ_MASK     (BT_GATT_PERM_READ | \
				 BT_GATT_PERM_READ_ENCRYPT | \
				 BT_GATT_PERM_READ_AUTHEN)
#define GATT_PERM_WRITE_MASK    (BT_GATT_PERM_WRITE | \
				 BT_GATT_PERM_WRITE_ENCRYPT | \
				 BT_GATT_PERM_WRITE_AUTHEN)

#ifndef CONFIG_BT_HRS_DEFAULT_PERM_RW_AUTHEN
#define CONFIG_BT_HRS_DEFAULT_PERM_RW_AUTHEN 0
#endif
#ifndef CONFIG_BT_HRS_DEFAULT_PERM_RW_ENCRYPT
#define CONFIG_BT_HRS_DEFAULT_PERM_RW_ENCRYPT 0
#endif
#ifndef CONFIG_BT_HRS_DEFAULT_PERM_RW
#define CONFIG_BT_HRS_DEFAULT_PERM_RW 0
#endif

#define HRS_GATT_PERM_DEFAULT (						\
	CONFIG_BT_HRS_DEFAULT_PERM_RW_AUTHEN ?				\
	(BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN) :	\
	CONFIG_BT_HRS_DEFAULT_PERM_RW_ENCRYPT ?				\
	(BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT) :	\
	(BT_GATT_PERM_READ | BT_GATT_PERM_WRITE))			\


#define BT_UUID \
	BT_UUID_DECLARE_16(0x80)

#define BT_UUID_MEASUREMENT \
	BT_UUID_DECLARE_16(0x2BFC)

#define BT_UUID_TIMESTAMP \
	BT_UUID_DECLARE_16(0x2A11)
	
struct bt_uuid_128 uuid = BT_UUID_INIT_128(0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0);



bool notif_enabled = false;
bool notif_enabled2 = false;

static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{

	notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	printk(" notifications for value %s\n", notif_enabled ? "enabled" : "disabled");
}
static void hrmc_ccc_cfg_changed_t(const struct bt_gatt_attr *attr, uint16_t value)
{

	notif_enabled2 = (value == BT_GATT_CCC_NOTIFY);

	printk(" notifications for timestamp %s\n", notif_enabled2 ? "enabled" : "disabled");
}



BT_GATT_SERVICE_DEFINE(sensor_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID),
	BT_GATT_CHARACTERISTIC(BT_UUID_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    HRS_GATT_PERM_DEFAULT),
	BT_GATT_CHARACTERISTIC(BT_UUID_TIMESTAMP, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed_t,
		    HRS_GATT_PERM_DEFAULT),
);


bool connected_check = false;




static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, 0x0f, 0x18),
};


static void bt_ready(void)
{
    int err;

    printk("Bluetooth initialized\n");



    struct bt_le_adv_param adv_param = {
        .options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
        .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
        .interval_max = BT_GAP_ADV_SLOW_INT_MAX,
        .id = BT_ID_DEFAULT,
        .sid = 0,
        .peer = NULL,
    };

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    printk("Advertising successfully started\n");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err 0x%02x)\n", err);
		connected_check = false;
    } else {
        printk("Connected\n");
        connected_check = true;
		char buff = 'A';
		bt_gatt_notify(NULL, &sensor_svc.attrs[1], &buff, 1);
    }
}


static void disconnected(struct bt_conn *conn, uint8_t err)
{
    printk("Disconnected (err 0x%02x)\n", err);
	connected_check = false;
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

bool config_connected_check = false;
void config_connected()
{
	printk("Connected\n");
	config_connected_check = true;

}
void config_disconnected()
{
	printk("Disconnected\n");
	config_connected_check = false;
}


struct bt_conn_cb config_cb_ble = {
	.connected = config_connected,
	.disconnected = config_disconnected,
};

const char name[] = "smart box config";
static const struct bt_data ad_conf[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, 0x0f, 0x18),
	BT_DATA(BT_DATA_NAME_COMPLETE, name, sizeof(name) - 1),
};


void configure_sensor_BLE()
{


	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	struct bt_le_adv_param adv_param = {
		.options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
		.interval_min = BT_GAP_ADV_SLOW_INT_MIN,
		.interval_max = BT_GAP_ADV_SLOW_INT_MAX,
		.id = BT_ID_DEFAULT,
		.sid = 0,
		.peer = NULL,
	};



	err = bt_le_adv_start(&adv_param, ad_conf, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
	//register a call back for when the device is connected
	bt_conn_cb_register(&config_cb_ble);

}



#endif // BLE_CONF_H