
#include "includes/i2c_sensors.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#include <stdlib.h>

#include "includes/flash_utils.h"

//TODO: make work with multiple values

int bcd_to_decimal(uint8_t bcd_value) {
    int tens = (bcd_value >> 4) & 0x0F;  // Extract tens digit
    int ones = bcd_value & 0x0F;          // Extract ones digit
    return tens * 10 + ones;             // Convert BCD to decimal
}



void write_data_fs(const struct nvs_fs *nvs_dev ,sensor *sn) {
    // Initialize NVS
    struct flash_entery data = 
    {
        .timestamp = sn->timestamp_32,
        .value = *((int32_t*) sn->values),
    };



    // Write sensor_data structure to flash memory
    int err = nvs_write(nvs_dev, sn->config._id, &data, sizeof(struct flash_entery));
    if (err<0) {
        printk("Error writing sensor data to flash (err %d)\n", err);
    }
}

void read_history_into(const struct nvs_fs *nvs_dev ,sensor *sn, struct flash_entery *readinto)
{
        int i =0;
        for(i = 0; i<sn->sucessful_read; i++)
        {
                nvs_read_hist(nvs_dev, sn->config._id, &readinto[i], sizeof(struct flash_entery), i);
          
                printk("RTC Data: %02x:%02x:%02x %02x/%02x/%02x\n", readinto[i].timestamp.sec, readinto[i].timestamp.min, readinto[i].timestamp.hour, readinto[i].timestamp.date, readinto[i].timestamp.month, readinto[i].timestamp.year);
        }

    
}


/* time stamp utilities*/


int8_t read_timestamp(const struct device *i2c_dev, struct time_stamp *readinto)
{
        int8_t write_buff = 0x0;
        uint8_t rtc_data[8];
        int err = 0;
        // Read data from RTC
        err = i2c_write(i2c_dev, &write_buff, 1, 82);

        printk("1: %d\n", err);

        if( err < 0 ) return -1;
        err = i2c_read(i2c_dev, &rtc_data, 7, 82);

        printk("2: %d\n", err);

        if(err < 0 ) return -1;

        printk("RTC Data: %02x:%02x:%02x %02x/%02x/%02x\n", rtc_data[2], rtc_data[1], rtc_data[0], rtc_data[3], rtc_data[4], rtc_data[5]);

        readinto->sec = rtc_data[0];
        readinto->min = rtc_data[1];
        readinto->hour = rtc_data[2];
        readinto->year = rtc_data[6];
        readinto->date = rtc_data[4];
        readinto->month = rtc_data[5];





        
        return 0;
}


int8_t get_timestamp(struct device *i2c_dev, uint8_t *readinto)
{
        
        if(!device_is_ready(i2c_dev))
        {
                printk("I2C device is not ready\n");
                return -1;
        }
        int err = 0;
        int8_t write_buff = 0x1B;
        uint8_t timestamp_data[4];
        
        
        err = i2c_write(i2c_dev, &write_buff, 1, 82);
        
        err = i2c_read(i2c_dev, timestamp_data, 4, 82);

        *readinto = (timestamp_data[0] |
                         (timestamp_data[1] << 8) |
                         (timestamp_data[2] << 16) |
                         (timestamp_data[3] << 24));
        return 0;
}

uint32_t read_unix_timestamp(struct device *i2c_dev)
{
    uint8_t reg = 0x1B;
    uint8_t timestamp_data[4]; // 4 bytes for the 32-bit Unix timestamp

    // Write the register address to read from
    if (i2c_write_read(i2c_dev, 82, &reg, 1, timestamp_data, sizeof(timestamp_data)) < 0) {
        printk("Failed to read timestamp from RTC\n");
        return 0;
    }

    // Convert the 4 bytes to a 32-bit Unix timestamp
    uint32_t timestamp = (timestamp_data[0] |
                         (timestamp_data[1] << 8) |
                         (timestamp_data[2] << 16) |
                         (timestamp_data[3] << 24));

    return timestamp;
}

