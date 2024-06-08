#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdlib.h>
#include "includes/i2c_sensors.h"
#include "includes/utils.h"


//TODO: add timestamp reading to each sensor

sensor* configure_sensor(sensor_config *configuration)
{
        if(configuration->type == I2C)
        {
                sensor *new_sensor = (sensor*)malloc(sizeof(sensor));
                new_sensor->sucessful_read = 0;
                new_sensor->config = *configuration;
                new_sensor->values = (int8_t*)malloc(2 * sizeof(uint8_t));
                new_sensor->timestamp = (time_stamp){0};
                return new_sensor;
        }
        else if(configuration->type == GPIO)
        {
                sensor *new_sensor = (sensor *) malloc(sizeof(sensor));
                new_sensor->sucessful_read = 0;
                new_sensor->config = *configuration;
                new_sensor->values = (int32_t*)malloc(sizeof(uint32_t));
                new_sensor->timestamp = (time_stamp) {0};
                return new_sensor;
        }
        
        return NULL;
}

int read_sensor(const void *drivers, sensor *sn, const struct adc_dt_spec channels[])
{
        if(sn->config.type == I2C)
        {
                if(drivers == NULL)
                {
                        
                        return -1;
                }
                for(int i = 0; i < sn->config.num_registers; i++)
                {
                        int err = 0;
                        err = i2c_write(drivers, &sn->config.reg_addr[i], 1, sn->config.adress);
                        if (err < 0 ) return -1;
                        err = i2c_read(drivers, &((uint8_t*)sn->values)[i], 1, sn->config.adress);
                }
                sn->sucessful_read++;
        }
        else if(sn->config.type == GPIO)
        {
                if(drivers == NULL)
                {
                        return -1;
                }
                if(channels == NULL)
                {
                        return -1;
                }
                
                int err;
                int32_t val_mv;
                int16_t raw_val = 0;
                
                struct adc_sequence sequence = {
                        .buffer = &raw_val,
                        /* buffer size in bytes, not number of samples */
                        .buffer_size = sizeof(raw_val),
                };

                err = adc_channel_setup_dt(&channels[sn->config.Pin-2]);
                (void)adc_sequence_init_dt(&channels[sn->config.Pin-2], &sequence);

		if (err < 0) {
                        printk("Could not setup channel #%d (%d)\n", sn->config._id, err);
                        return 0;
		}
                

                err = adc_read_dt(&channels[sn->config.Pin-2], &sequence);
                if (err < 0) {
                        printk("Could not read (%d)\n", err);
                }

                val_mv = (int32_t)(raw_val);

                err = adc_raw_to_millivolts_dt(&channels[sn->config.Pin-2], &val_mv);
                
                ((int32_t*)sn->values)[0] = val_mv;

                sn->sucessful_read++;

        }
        else if(sn->config.type == SPI)
        {
                //!TODO: read the value from the SPI bus
                return 0;
        }
        else if(sn->config.type == UART)
        {
                //!TODO: read the value from the UART bus
                return 0;
        }
        else
        {
                return -1;
        }
        //read_timestamp(drivers, &sn->timestamp);
        sn->timestamp_32 = read_unix_timestamp((struct device *) drivers);
        printk("sn->timestamp_32 = %d\n", sn->timestamp_32);

        return 0;
}