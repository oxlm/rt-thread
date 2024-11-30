
#include <rtdevice.h>
#include <rtthread.h>

typedef struct st_rm_air_sensor_single_data
{
    uint32_t integer_part;             ///< Integer part of sensor data.
    uint32_t decimal_part;             ///< Decimal part of sensor data.
} rm_air_sensor_single_data_t;

/** AIR SENSOR data block */
struct rrh62000_data
{
    uint32_t status;
    rm_air_sensor_single_data_t nc_0p3;      ///< Number concentration of particle size 0.3 um - 10 um [1/cm3]
    rm_air_sensor_single_data_t nc_0p5;      ///< Number concentration of particle size 0.5 um - 10 um [1/cm3]
    rm_air_sensor_single_data_t nc_1;        ///< Number concentration of particle size 1 um - 10 um [1/cm3]
    rm_air_sensor_single_data_t nc_2p5;      ///< Number concentration of particle size 2.5 um - 10 um [1/cm3]
    rm_air_sensor_single_data_t nc_4;        ///< Number concentration of particle size 4 um - 10 um [1/cm3]
    rm_air_sensor_single_data_t pm1_1;       ///< Mass concentration of particle size 0.3 um - 1 um with reference to KCl particle [um/cm3]
    rm_air_sensor_single_data_t pm2p5_1;     ///< Mass concentration of particle size 0.3 um - 2.5 um with reference to KCl particle [um/cm3]
    rm_air_sensor_single_data_t pm10_1;      ///< Mass concentration of particle size 0.3 um - 10 um with reference to KCl particle [um/cm3]
    rm_air_sensor_single_data_t pm1_2;       ///< Mass concentration of particle size 0.3 um - 1 um with reference to cigarette smoke [um/cm3]
    rm_air_sensor_single_data_t pm2p5_2;     ///< Mass concentration of particle size 0.3 um - 2.5 um with reference to cigarette smoke [um/cm3]
    rm_air_sensor_single_data_t pm10_2;      ///< Mass concentration of particle size 0.3 um - 10 um with reference to cigarette smoke [um/cm3]
    rm_air_sensor_single_data_t temperature; ///< Temperature [Celsius]
    rm_air_sensor_single_data_t humidity;    ///< Humidity [%RH]
    rm_air_sensor_single_data_t tvoc;        ///< Total volatile organic compounds (TVOC) concentrations [mg/m3]
    rm_air_sensor_single_data_t eco2;        ///< Estimated carbon dioxide (eCO2) level [ppm]
    rm_air_sensor_single_data_t iaq;         ///< Indoor Air Quality level according to UBA
    rm_air_sensor_single_data_t rel_iaq;     ///< Relative IAQ
};

rt_err_t rrh62000_read_measured_value(struct rt_i2c_bus_device *bus, struct rrh62000_data *data);
rt_err_t rrh62000_read_data_status(struct rt_i2c_bus_device *bus, rt_bool_t *isDataReady);
rt_err_t rrh62000_sleep_set(struct rt_i2c_bus_device *bus, rt_bool_t isSleep);
rt_err_t rrh62000_sleep_get(struct rt_i2c_bus_device *bus, rt_bool_t *isSleep);
rt_err_t rrh62000_start_dust_cleaning(struct rt_i2c_bus_device *bus);
rt_err_t rrh62000_reset(struct rt_i2c_bus_device *bus);
rt_err_t rrh62000_moving_average_set(struct rt_i2c_bus_device *bus, uint8_t time);
rt_err_t rrh62000_moving_average_get(struct rt_i2c_bus_device *bus, uint8_t *time);
rt_err_t rrh62000_clean_interval_time_set(struct rt_i2c_bus_device *bus, uint16_t time);
rt_err_t rrh62000_clean_interval_time_get(struct rt_i2c_bus_device *bus, uint16_t *time);
rt_err_t rrh62000_auto_clean_time_set(struct rt_i2c_bus_device *bus, uint8_t time);
rt_err_t rrh62000_auto_clean_time_get(struct rt_i2c_bus_device *bus, uint8_t *time);
rt_err_t rrh62000_fan_speed_set(struct rt_i2c_bus_device *bus, uint8_t speed);
rt_err_t rrh62000_fan_speed_get(struct rt_i2c_bus_device *bus, uint8_t *speed);
rt_err_t rrh62000_read_mox_resistance(struct rt_i2c_bus_device *bus, uint32_t *data);

struct rrh62000_uid{
    char uid[6];
};

rt_err_t rrh62000_read_uid(struct rt_i2c_bus_device *bus, struct rrh62000_uid *data);

struct rrh62000_algoversion{
    char major;
    char minor;
    char patch;
};

rt_err_t rrh62000_read_algoritm_verison(struct rt_i2c_bus_device *bus, struct rrh62000_algoversion *data);
rt_err_t rrh62000_read_TVOC_sensor_clean_status(struct rt_i2c_bus_device *bus, rt_bool_t *isComplete);

struct rrh62000_firmwareversion{
    char major;
    char minor;
};

rt_err_t rrh62000_read_firmware_verison(struct rt_i2c_bus_device *bus, struct rrh62000_firmwareversion *data);