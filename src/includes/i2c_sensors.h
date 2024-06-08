
#ifndef I2C_SENSORS_H
#define I2C_SENSORS_H

#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>


typedef struct time_stamp
{
        uint8_t sec;
        uint8_t min;
        uint8_t hour;
        uint8_t day;
        uint8_t date;
        uint8_t month;
        uint8_t year;
} time_stamp;


typedef enum type {
        I2C=0,
        SPI=1,
        GPIO=2,
        UART=3
} sensor_type;

typedef enum AINPUT{
        AIN1=0,
        AIN2=1,
        AIN3=2,
        AIN4=3,
        AIN5=4,
        AIN6=5,
        AIN7=6,
} AINPIN;

typedef struct sensor_config
{
        uint16_t adress;
        uint8_t _id;
        uint8_t reg_addr[8];
        uint8_t num_registers;
        uint8_t Pin; //for a GPIO sensor, give the relevant ADC pin, for an SPI give the CS pin, for a UART or i2c sensor, give -1 (255)
        struct adc_dt_spec *channel; //for a GPIO sensor, give the relevant ADC channel, for an SPI, UART or i2c sensor, give NULL
        sensor_type type;
        size_t value_size;
        uint8_t flash_id;

} sensor_config;

typedef struct sensor_reading
{
        sensor_config config;
        void *values;
        time_stamp timestamp;
        uint8_t sucessful_read;
        uint32_t timestamp_32;
} sensor;

/***********************************************************
//@brief: configure the sensor with the given configuration
//@param: sensor_config *configuration: pointer to the configuration of the sensor, can safely be freed after the sensor is configured
//@return: pointer to the sensor struct
*********************************************************/
sensor* configure_sensor(sensor_config *configuration);

/***********************************************************
 * @brief: read the sensor and store the values in the sensor struct
 * @param: sensor *sensor: pointer to the sensor struct 
 * @return: 0 if the sensor was read successfully, negative error code if the sensor could not be read
 *************************************************************/
int read_sensor( const void *i2c_dev, sensor *sensor, const struct adc_dt_spec channels[]);







#endif