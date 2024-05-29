#ifndef FLASH_UTILS_H
#define FLASH_UTILS_H

#include "i2c_sensors.h"


struct flash_entery
{
    time_stamp timestamp;
    int16_t value;
};

void read_history_into(const struct nvs_fs *nvs_dev ,sensor *sn, struct flash_entery *readinto);

void write_data_fs(const struct nvs_fs *nvs_dev ,sensor *sn);



#endif