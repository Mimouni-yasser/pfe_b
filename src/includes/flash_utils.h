#ifndef FLASH_UTILS_H
#define FLASH_UTILS_H

#include "i2c_sensors.h"


struct flash_entery
{
    uint32_t timestamp;
    int16_t value;
};

void read_history_into( struct nvs_fs *nvs_dev, sensor *sn, struct flash_entery *readinto, size_t offset, size_t entries_to_read);

void write_data_fs(struct nvs_fs *nvs_dev ,sensor *sn);



#endif