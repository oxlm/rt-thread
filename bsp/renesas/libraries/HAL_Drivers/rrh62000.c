#include "rrh62000.h"

#define RRH62000_ADDR               0x69

/* Definitions of Timeout */
#define RM_RRH62000_TIMEOUT                       (100)

/* Definitions of Wait Time */
#define RM_RRH62000_WAIT_TIME_1000                (1000)

/* Definitions of Retry max counts */
#define RM_RRH62000_RETRY_MAX_COUNTS              (5)
#define RM_RRH62000_RETRY_ZMOD_CLEANING_COUNTS    (60 + 10) // ZMOD cleaning time (60s) + buffer (10s)

/* Definitions of Command */
#define RM_RRH62000_COMMAND_READ                  (0x00)
#define RM_RRH62000_COMMAND_DATA                  (0x40)
#define RM_RRH62000_COMMAND_RESET                 (0x52)
#define RM_RRH62000_COMMAND_MAVE                  (0x53)
#define RM_RRH62000_COMMAND_SPEEDFAN              (0x63)
#define RM_RRH62000_COMMAND_ARGVER                (0x73)
#define RM_RRH62000_COMMAND_CSTATUS               (0x74)
#define RM_RRH62000_COMMAND_FWVAR                 (0x75)

/* Definitions of Reset */
#define RM_RRH62000_RESET_VALUE                   (0x81)

/* Definitions of Cleaning Status */
#define RM_RRH62000_ZMOD_CLEAN_NOT_COMPLETE       (0x00)
#define RM_RRH62000_ZMOD_CLEAN_COMPLETE           (0x01)

/* Definitions of data size */
#define RM_RRH62000_LEN_MEASUREMENT_DATA          (37)

/* Definitions of Mask */
#define RM_RRH62000_STATUS_MASK                   (0x01)
#define RM_RRH62000_HIGH_CONCENTRATION_MASK       (0x01)
#define RM_RRH62000_DUST_ACCUMULATION_MASK        (0x02)
#define RM_RRH62000_FAN_SPEED_MASK                (0x04)
#define RM_RRH62000_FAN_MASK                      (0x08)

/* Definitions of Position */
#define RM_RRH62000_POSITION_STATUS               (0)
#define RM_RRH62000_POSITION_NC_0P3               (2)
#define RM_RRH62000_POSITION_TEMPERATURE          (24)
#define RM_RRH62000_POSITION_ECO2                 (30)
#define RM_RRH62000_POSITION_IAQ                  (32)

/* Definitions of Calculation */
#define RM_RRH62000_CALC_CRC_INITIAL_VALUE        (0xFF)
#define RM_RRH62000_CALC_CRC_DATA_LENGTH          (36)
#define RM_RRH62000_CALC_DATA_STEP                (2)
#define RM_RRH62000_CALC_CRC_8BITS_LENGTH         (8)
#define RM_RRH62000_CACL_CRC_0X80                 (0x80)
#define RM_RRH62000_CACL_CRC_MASK_MSB             (0x80)
#define RM_RRH62000_CALC_CRC_POLYNOMIAL           (0x31)
#define RM_RRH62000_CALC_CRC_FINAL_XOR            (0x00)
#define RM_RRH62000_CALC_DECIMAL_VALUE_100        (100)
#define RM_RRH62000_CALC_DECIMAL_VALUE_10         (10)

/* Definitions of Shift */
#define RM_RRH62000_SHIFT_24                      (24)
#define RM_RRH62000_SHIFT_16                      (16)
#define RM_RRH62000_SHIFT_8                       (8)


static rt_err_t rrh62000_read(struct rt_i2c_bus_device *bus, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t        tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = RRH62000_ADDR;      /* Slave address */
    msgs[0].flags = RT_I2C_WR; /* Write flag */
    msgs[0].buf   = &tmp;      /* Slave register address */
    msgs[0].len   = 1;         /* Number of bytes sent */

    msgs[1].addr  = RRH62000_ADDR;      /* Slave address */
    msgs[1].flags = RT_I2C_RD; /* Read flag */
    msgs[1].buf   = data;      /* Read data pointer */
    msgs[1].len   = len;       /* Number of bytes read */

    if (rt_i2c_transfer(bus, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t rrh62000_write(struct rt_i2c_bus_device *bus, uint8_t reg, uint8_t *data, uint16_t len)
{
    struct rt_i2c_msg msgs[2];
    uint8_t           data1[5];
    uint16_t          i;

    data1[0] = reg;
    for (i = 0; i < len; i++)
    {
        data1[i + 1] = data[i];
    }

    msgs[0].addr  = RRH62000_ADDR;      /* Slave address */
    msgs[0].flags = RT_I2C_WR; /* Write flag */
    msgs[0].buf   = data1;     /* Slave register address */
    msgs[0].len   = len + 1;   /* Number of bytes sent */

    if (rt_i2c_transfer(bus, msgs, 1) != 1)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t rm_rrh62000_crc_execute (const uint8_t * const p_raw_data)
{
    uint8_t         crc;
    const uint8_t * p_input_data;
    uint8_t         i;
    uint8_t         j;

    /* Set pointer to input data */
    p_input_data = &p_raw_data[0];

    /* Set initial value */
    crc = RM_RRH62000_CALC_CRC_INITIAL_VALUE;

    /* Execute CRC */
    for (i = 0; i < RM_RRH62000_CALC_CRC_DATA_LENGTH; i++)
    {
        /* Calculate XOR with input value */
        crc ^= *p_input_data;

        for (j = 0; j < RM_RRH62000_CALC_CRC_8BITS_LENGTH; j++)
        {
            if (RM_RRH62000_CACL_CRC_0X80 == (crc & RM_RRH62000_CACL_CRC_MASK_MSB))
            {
                /* If MSB is 1, calculate XOR with the polynomial. */
                crc = (uint8_t) (crc << 1) ^ RM_RRH62000_CALC_CRC_POLYNOMIAL;
            }
            else
            {
                crc <<= 1;
            }
        }

        p_input_data++;
    }

    /* Final XOR */
    crc ^= RM_RRH62000_CALC_CRC_FINAL_XOR;
    if(p_raw_data[RM_RRH62000_CALC_CRC_DATA_LENGTH] == crc)
    {
        return RT_EOK;    
    }
    else
    {
        return -RT_ERROR;    
    }
}

static rt_err_t RM_RRH62000_DataCalculate (uint8_t * p_raw_data, struct rrh62000_data *p_rrh62000_data)
{
    rt_err_t err = RT_EOK;
    rt_uint32_t tmp_u32;
    rm_air_sensor_single_data_t * p_sensor_data;
    rt_uint32_t position;

    /* Execute CRC */
    err = rm_rrh62000_crc_execute(p_raw_data);
    if(err != RT_EOK)
    {
        return err;    
    }

    /* Set status data */
    p_rrh62000_data->status =
        (uint32_t) (((uint32_t) p_raw_data[1] << RM_RRH62000_SHIFT_8) | (uint32_t) p_raw_data[0]);

    /* Set measurement results */
    p_sensor_data = &p_rrh62000_data->nc_0p3;
    for (position = RM_RRH62000_POSITION_NC_0P3;
         position < RM_RRH62000_LEN_MEASUREMENT_DATA;
         position += RM_RRH62000_CALC_DATA_STEP)
    {
        /* Calculate sensor data from measurement results (big-endian). */
        tmp_u32 = (uint32_t) (((uint32_t) p_raw_data[position] << RM_RRH62000_SHIFT_8) |
                              (uint32_t) p_raw_data[position + 1]);
        if (RM_RRH62000_POSITION_TEMPERATURE > position)
        {
            /* NC_x and PMx_x data. One decimal place. */
            p_sensor_data->integer_part = tmp_u32 / RM_RRH62000_CALC_DECIMAL_VALUE_10;
            p_sensor_data->decimal_part = tmp_u32 % RM_RRH62000_CALC_DECIMAL_VALUE_10;
        }
        else if ((RM_RRH62000_POSITION_ECO2 > position) || (RM_RRH62000_POSITION_IAQ == position))
        {
            /* Temperature, humidity, TVOC and IAQ data. Two decimal places. */
            p_sensor_data->integer_part = tmp_u32 / RM_RRH62000_CALC_DECIMAL_VALUE_100;
            p_sensor_data->decimal_part = tmp_u32 % RM_RRH62000_CALC_DECIMAL_VALUE_100;
        }
        else
        {
            /* eCO2 and Relative IAQ data. These data does not have a decimal part. */
            p_sensor_data->integer_part = tmp_u32;
            p_sensor_data->decimal_part = 0;
        }

        p_sensor_data++;
    }

    return RT_EOK;
}

rt_err_t rrh62000_read_measured_value(struct rt_i2c_bus_device *bus, struct rrh62000_data *data)
{
    rt_uint8_t buf[37];
    rt_uint32_t len = 37;

    if(rrh62000_read(bus, RM_RRH62000_COMMAND_READ, buf, len) != RT_EOK) {
        return -RT_ERROR;
    }

    return RM_RRH62000_DataCalculate(buf, data);
}

rt_err_t rrh62000_read_data_status(struct rt_i2c_bus_device *bus, rt_bool_t *isDataReady)
{
    rt_uint8_t buf[1];
    if(rrh62000_read(bus, RM_RRH62000_COMMAND_DATA, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    *isDataReady = buf[0];

    return RT_EOK;
}

rt_err_t rrh62000_sleep_set(struct rt_i2c_bus_device *bus, rt_bool_t isSleep)
{
    rt_uint8_t buf[1];

    if(isSleep)
    {
        buf[0] = 0x00;
    }
    else
    {
        buf[0] = 0x80;
    }

    if(rrh62000_write(bus, 0x50, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    return RT_EOK;
}

rt_err_t rrh62000_sleep_get(struct rt_i2c_bus_device *bus, rt_bool_t *isSleep)
{
    rt_uint8_t buf[1];

    if(rrh62000_read(bus, 0x50, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    if(buf[0])
    {
        *isSleep = 0;
    }
    else
    {
        *isSleep = 1;
    }

    return RT_EOK;
}

rt_err_t rrh62000_start_dust_cleaning(struct rt_i2c_bus_device *bus)
{
    rt_uint8_t buf[1] = {0x01};
    if(rrh62000_write(bus, 0x51, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    return RT_EOK;
}

rt_err_t rrh62000_reset(struct rt_i2c_bus_device *bus)
{
    rt_uint8_t buf[1] = {0x81};
    if(rrh62000_write(bus, RM_RRH62000_COMMAND_RESET, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    return RT_EOK;
}

rt_err_t rrh62000_moving_average_set(struct rt_i2c_bus_device *bus, uint8_t time)
{
    rt_uint8_t buf[1];

    if(time > 60)
    {
        time = 60;
    }
    else if(time < 1)
    {
        time = 1;
    }
    buf[0] = time;

    if(rrh62000_write(bus, RM_RRH62000_COMMAND_MAVE, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    return RT_EOK;
}

rt_err_t rrh62000_moving_average_get(struct rt_i2c_bus_device *bus, uint8_t *time)
{
    rt_uint8_t buf[1];

    if(rrh62000_read(bus, RM_RRH62000_COMMAND_MAVE, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    *time = buf[0];

    return RT_EOK;
}

rt_err_t rrh62000_clean_interval_time_set(struct rt_i2c_bus_device *bus, uint16_t time)
{
    rt_uint8_t buf[1];

    if(time > 60480)
    {
        time = 60480;
    }
    buf[0] = (time >> 8);
    if(rrh62000_write(bus, 0x5A, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    buf[0] = (time & 0x00FF);
    if(rrh62000_write(bus, 0x5B, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }
    return RT_EOK;
}

rt_err_t rrh62000_clean_interval_time_get(struct rt_i2c_bus_device *bus, uint16_t *time)
{
    rt_uint8_t buf[2];

    if(rrh62000_read(bus, 0x5A, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    if(rrh62000_read(bus, 0x5B, buf + 1, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    *time = ((uint16_t)buf[0] << 8) + buf[1];

    return RT_EOK;
}

rt_err_t rrh62000_auto_clean_time_set(struct rt_i2c_bus_device *bus, uint8_t time)
{
    rt_uint8_t buf[1];

    if(time > 60)
    {
        time = 60;
    }
    buf[0] = time;

    if(rrh62000_write(bus, 0x5C, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    return RT_EOK;
}

rt_err_t rrh62000_auto_clean_time_get(struct rt_i2c_bus_device *bus, uint8_t *time)
{
    rt_uint8_t buf[1];

    if(rrh62000_read(bus, 0x5C, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    *time = buf[0];

    return RT_EOK;
}

rt_err_t rrh62000_fan_speed_set(struct rt_i2c_bus_device *bus, uint8_t speed)
{
    rt_uint8_t buf[1];

    if(speed > 100)
    {
        speed = 100;
    }
    else if(speed < 60)
    {
        speed = 60;
    }
    buf[0] = speed;

    if(rrh62000_write(bus, RM_RRH62000_COMMAND_SPEEDFAN, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    return RT_EOK;
}

rt_err_t rrh62000_fan_speed_get(struct rt_i2c_bus_device *bus, uint8_t *speed)
{
    rt_uint8_t buf[1];

    if(rrh62000_read(bus, RM_RRH62000_COMMAND_SPEEDFAN, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    *speed = buf[0];

    return RT_EOK;
}

rt_err_t rrh62000_read_mox_resistance(struct rt_i2c_bus_device *bus, uint32_t *data)
{
    rt_uint8_t buf[4];
    if(rrh62000_read(bus, 0x71, buf, 4) != RT_EOK) {
        return -RT_ERROR;
    }

    *data  = (uint32_t)buf[0] << 24;
    *data += (uint32_t)buf[1] << 16;
    *data += (uint32_t)buf[2] << 8;
    *data += (uint32_t)buf[3] << 0;

    return RT_EOK;
}

rt_err_t rrh62000_read_uid(struct rt_i2c_bus_device *bus, struct rrh62000_uid *data)
{
    rt_uint8_t buf[6];
    if(rrh62000_read(bus, 0x72, buf, 6) != RT_EOK) {
        return -RT_ERROR;
    }

    rt_memcpy(&data->uid, buf, 6);

    return RT_EOK;
}

rt_err_t rrh62000_read_algoritm_verison(struct rt_i2c_bus_device *bus, struct rrh62000_algoversion *data)
{
    rt_uint8_t buf[3];
    if(rrh62000_read(bus, RM_RRH62000_COMMAND_ARGVER, buf, 3) != RT_EOK) {
        return -RT_ERROR;
    }

    data->major = buf[0];
    data->minor = buf[1];
    data->patch = buf[2];

    return RT_EOK;
}

rt_err_t rrh62000_read_TVOC_sensor_clean_status(struct rt_i2c_bus_device *bus, rt_bool_t *isComplete)
{
    rt_uint8_t buf[1];
    if(rrh62000_read(bus, RM_RRH62000_COMMAND_CSTATUS, buf, 1) != RT_EOK) {
        return -RT_ERROR;
    }

    *isComplete = buf[0];

    return RT_EOK;
}

rt_err_t rrh62000_read_firmware_verison(struct rt_i2c_bus_device *bus, struct rrh62000_firmwareversion *data)
{
    rt_uint8_t buf[2];
    if(rrh62000_read(bus, RM_RRH62000_COMMAND_FWVAR, buf, 2) != RT_EOK) {
        return -RT_ERROR;
    }

    data->major = buf[0];
    data->minor = buf[1];

    return RT_EOK;
}
