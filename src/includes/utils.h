#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/smf.h>

int bcd_to_decimal(uint8_t bcd_value);
// int8_t read_timestamp(struct device *dev_i2c, struct time_stamp *readinto);
// int8_t get_timestamp(struct device *dev_i2c, struct time_stamp *readinto);
uint32_t read_unix_timestamp(struct device *i2c_dev);


void delete_entries_with_id(const struct nvs_fs *fs, uint16_t target_id);


// Function to check if GPIO 1.15 is high


#endif // UTILS_H