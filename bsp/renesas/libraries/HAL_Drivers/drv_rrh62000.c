#include <drivers/sensor.h>
#include "hal_data.h"
#include "rrh62000.h"

struct sensor_device {
    rt_bool_t openCount;
    struct rt_i2c_bus_device * dev;
    struct rrh62000_data data;
};

static rt_ssize_t _get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    struct sensor_device *dev = sensor->parent.user_data;
    rt_bool_t isDataReady = RT_FALSE;

    switch(sensor->info.type)
    {
    case RT_SENSOR_CLASS_TEMP:
    case RT_SENSOR_CLASS_HUMI:
    case RT_SENSOR_CLASS_TVOC:
    case RT_SENSOR_CLASS_ECO2:
    case RT_SENSOR_CLASS_IAQ:
    case RT_SENSOR_CLASS_DUST:
        if((rrh62000_read_data_status(dev->dev, &isDataReady) == RT_EOK) && (isDataReady))
        {
            rrh62000_read_measured_value(dev->dev, &dev->data);
        }
        break;
    default:
        return 0;
    }

    switch(sensor->info.type)
    {
    case RT_SENSOR_CLASS_TEMP:
        data->data.temp = (dev->data.temperature.integer_part << 16) + dev->data.temperature.decimal_part;
        break;
        
    case RT_SENSOR_CLASS_HUMI:
        data->data.temp = (dev->data.humidity.integer_part << 16) + dev->data.humidity.decimal_part;
        break;

    case RT_SENSOR_CLASS_TVOC:
        data->data.temp = (dev->data.tvoc.integer_part << 16) + dev->data.tvoc.decimal_part;
        break;

    case RT_SENSOR_CLASS_ECO2:
        data->data.temp = (dev->data.eco2.integer_part << 16) + dev->data.eco2.decimal_part;
        break;

    case RT_SENSOR_CLASS_IAQ:
        data->data.temp = (dev->data.iaq.integer_part << 16) + dev->data.iaq.decimal_part;
        break;

    case RT_SENSOR_CLASS_DUST:
        data->data.temp = (dev->data.nc_2p5.integer_part << 16) + dev->data.nc_2p5.decimal_part;
        break;

    default:
        goto RET;
        break;
    }

    data->timestamp = rt_sensor_get_ts();
    return 1;

RET:
    return 0;
}

static rt_ssize_t fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
        return _get_data(sensor, buf);
    else
        return 0;
}

static rt_err_t _set_power(struct rt_sensor_device *sensor, void *args)
{
    struct sensor_device *dev = sensor->parent.user_data;
    rt_uint8_t power = *(uint8_t *)args;
    //fsp_err_t ret;
    rt_err_t ret;
    rt_bool_t isTVOCCLean = RT_FALSE;
    int count = 70; // 70s

    if(power == RT_SENSOR_POWER_DOWN)
    {
        if(dev->openCount != 0)
        {
            dev->openCount--;
        }

        return RT_EOK;
    }

    if(dev->openCount == 0)
    {
        do
        {
            if(rrh62000_read_TVOC_sensor_clean_status(dev->dev, &isTVOCCLean) == RT_EOK)
            {
                if(isTVOCCLean == RT_FALSE)
                {
                    rt_thread_mdelay(1000);
                    count--;
                }
            }
        }while((isTVOCCLean == RT_FALSE) && (count--));

        if(count == 0)
        {
            return -RT_ERROR;
        }

        ret = rrh62000_reset(dev->dev);
        if(ret != RT_EOK)
        {
            rt_kprintf("open failed %d\n\r",ret);
            return -RT_ERROR;
        }
        
        rt_thread_mdelay(1000);

        ret = rrh62000_moving_average_set(dev->dev, 1);
        ret = rrh62000_fan_speed_set(dev->dev, 60);
    }
    dev->openCount++;

    return RT_EOK;
}

static rt_err_t control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    
    switch (cmd)
    {
        case RT_SENSOR_CTRL_GET_ID:
            // TODO:  get device id
            // result = xxxx(sensor, args);
            break;
        case RT_SENSOR_CTRL_GET_INFO:
            // TODO:  get info
            // result = xxxx(sensor, args);
            break;
        case RT_SENSOR_CTRL_SET_RANGE:
            // TODO: set test range
            // result = xxxx(sensor, args);
            break;
        case RT_SENSOR_CTRL_SET_ODR:
            // TODO: set frequency
            // result = xxxx(sensor, args);
            break;
        case RT_SENSOR_CTRL_SET_MODE:
            // TODO: set work mode
            // result = xxxx(sensor, args);
            break;
        case RT_SENSOR_CTRL_SET_POWER:
            // TODO: set power mode
            result = _set_power(sensor, args);
            break;

        case RT_SENSOR_CTRL_SELF_TEST:
            // TODO: process self test
            // result = xxxx(sensor);
            break;
        default:
            return -RT_ERROR;
    }
    
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    fetch_data,
    control
};

struct sensor_device * rrh62000_param_init(void)
{
    struct sensor_device *dev;
    struct rrh62000_firmwareversion fwVersion;
    struct rrh62000_algoversion algoVersion;

    dev = rt_calloc(1, sizeof(struct sensor_device));
    if(!dev)
    {
        goto exit;
    }
    rt_memset(dev, 0x00, sizeof(struct sensor_device));
    dev->dev = (struct rt_i2c_bus_device *)rt_device_find("i2c1");
    if(dev->dev == RT_NULL)
    {
        goto exit;
    }


    if(rrh62000_read_firmware_verison(dev->dev, &fwVersion) != RT_EOK)
    {
        goto exit;
    }

    if(rrh62000_read_algoritm_verison(dev->dev, &algoVersion) != RT_EOK)
    {
        goto exit;
    }

    rt_kprintf("rrh62000 firmware version %d.%d\n\r", fwVersion.major, fwVersion.minor);
    rt_kprintf("rrh62000 algoritm version %d.%d.%d\n\r", algoVersion.major, algoVersion.minor, algoVersion.patch);

    return dev;

exit:
    if(dev)
        rt_free(dev);
    return RT_NULL;
}

#ifdef BSP_USING_RRH62000_TEMP
rt_err_t rrh62000_register_temperature(const char *name, struct rt_sensor_config *cfg, struct sensor_device *dev)
{
    rt_sensor_t sensor = RT_NULL;
    rt_int8_t result;

    /* sensor register */
    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        goto __exit;
    
    sensor->info.type       = RT_SENSOR_CLASS_TEMP; // Set real type
    sensor->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN; // Set real vendor
    sensor->info.model      = name;  // set real model name
    sensor->info.unit       = RT_SENSOR_UNIT_DCELSIUS; // set to real unit flag
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C; // Set interface type
    sensor->info.range_max  = 0xFFFF; // Set to range max
    sensor->info.range_min  = 0x0000; // Set to range min
    sensor->info.period_min = 50; // Set frequency

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDONLY, dev);
    if (result != RT_EOK)
    {
        goto __exit;
    }

    return RT_EOK;

__exit:
    if (sensor)
        rt_free(sensor);

    return -RT_ERROR;
}
#endif

#ifdef BSP_USING_RRH62000_HUMI
rt_err_t rrh62000_register_humidity(const char *name, struct rt_sensor_config *cfg, struct sensor_device *dev)
{
    rt_sensor_t sensor = RT_NULL;
    rt_int8_t result;

    /* sensor register */
    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        goto __exit;
    
    sensor->info.type       = RT_SENSOR_CLASS_HUMI; // Set real type
    sensor->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN; // Set real vendor
    sensor->info.model      = name;  // set real model name
    sensor->info.unit       = RT_SENSOR_UNIT_PERMILLAGE; // set to real unit flag
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C; // Set interface type
    sensor->info.range_max  = 0xFFFF; // Set to range max
    sensor->info.range_min  = 0x0000; // Set to range min
    sensor->info.period_min = 50; // Set frequency

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDONLY, dev);
    if (result != RT_EOK)
    {
        goto __exit;
    }

    return RT_EOK;

__exit:
    if (sensor)
        rt_free(sensor);

    return -RT_ERROR;
}
#endif

#ifdef BSP_USING_RRH62000_TVOC
rt_err_t rrh62000_register_TVOC(const char *name, struct rt_sensor_config *cfg, struct sensor_device *dev)
{
    rt_sensor_t sensor = RT_NULL;
    rt_int8_t result;

    /* sensor register */
    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        goto __exit;
    
    sensor->info.type       = RT_SENSOR_CLASS_TVOC; // Set real type
    sensor->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN; // Set real vendor
    sensor->info.model      = name;  // set real model name
    sensor->info.unit       = RT_SENSOR_UNIT_MGM3; // set to real unit flag
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C; // Set interface type
    sensor->info.range_max  = 0xFFFF; // Set to range max
    sensor->info.range_min  = 0x0000; // Set to range min
    sensor->info.period_min = 50; // Set frequency

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDONLY, dev);
    if (result != RT_EOK)
    {
        goto __exit;
    }

    return RT_EOK;

__exit:
    if (sensor)
        rt_free(sensor);

    return -RT_ERROR;
}
#endif

#ifdef BSP_USING_RRH62000_DUST
rt_err_t rrh62000_register_Dust(const char *name, struct rt_sensor_config *cfg, struct sensor_device *dev)
{
    rt_sensor_t sensor = RT_NULL;
    rt_int8_t result;

    /* sensor register */
    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        goto __exit;
    
    sensor->info.type       = RT_SENSOR_CLASS_DUST; // Set real type
    sensor->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN; // Set real vendor
    sensor->info.model      = name;  // set real model name
    sensor->info.unit       = RT_SENSOR_UNIT_MGM3; // set to real unit flag
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C; // Set interface type
    sensor->info.range_max  = 0xFFFF; // Set to range max
    sensor->info.range_min  = 0x0000; // Set to range min
    sensor->info.period_min = 50; // Set frequency

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDONLY, dev);
    if (result != RT_EOK)
    {
        goto __exit;
    }

    return RT_EOK;

__exit:
    if (sensor)
        rt_free(sensor);

    return -RT_ERROR;
}
#endif

#ifdef BSP_USING_RRH62000_ECO2
rt_err_t rrh62000_register_ECO2(const char *name, struct rt_sensor_config *cfg, struct sensor_device *dev)
{
    rt_sensor_t sensor = RT_NULL;
    rt_int8_t result;

    /* sensor register */
    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        goto __exit;
    
    sensor->info.type       = RT_SENSOR_CLASS_ECO2; // Set real type
    sensor->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN; // Set real vendor
    sensor->info.model      = name;  // set real model name
    sensor->info.unit       = RT_SENSOR_UNIT_PPM; // set to real unit flag
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C; // Set interface type
    sensor->info.range_max  = 0xFFFF; // Set to range max
    sensor->info.range_min  = 0x0000; // Set to range min
    sensor->info.period_min = 50; // Set frequency

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDONLY, dev);
    if (result != RT_EOK)
    {
        goto __exit;
    }

    return RT_EOK;

__exit:
    if (sensor)
        rt_free(sensor);

    return -RT_ERROR;
}
#endif

#ifdef BSP_USING_RRH62000_IAQ
rt_err_t rrh62000_register_IAQ(const char *name, struct rt_sensor_config *cfg, struct sensor_device *dev)
{
    rt_sensor_t sensor = RT_NULL; 
    rt_int8_t result;

    /* sensor register */
    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        goto __exit;
    
    sensor->info.type       = RT_SENSOR_CLASS_IAQ; // Set real type
    sensor->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN; // Set real vendor
    sensor->info.model      = name;  // set real model name
    sensor->info.unit       = RT_SENSOR_UNIT_NONE; // set to real unit flag
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C; // Set interface type
    sensor->info.range_max  = 0xFFFF; // Set to range max
    sensor->info.range_min  = 0x0000; // Set to range min
    sensor->info.period_min = 50; // Set frequency

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDONLY, dev);
    if (result != RT_EOK)
    {
        goto __exit;
    }

    return RT_EOK;

__exit:
    if (sensor)
        rt_free(sensor);

    return -RT_ERROR;
}
#endif

static int rrh62000_device_register(void)
{
    rt_int8_t result;
    struct sensor_device *dev;
    const char *name = "rrh62000";
    struct rt_sensor_config      cfg = {
        .mode = RT_SENSOR_MODE_POLLING,
        .power = RT_SENSOR_POWER_DOWN,
    };

    dev = rrh62000_param_init();
    if(dev == RT_NULL)
    {
        goto __exit;
    }

#ifdef BSP_USING_RRH62000_TEMP
    result = rrh62000_register_temperature(name, &cfg, dev);
    if (result != RT_EOK)
    {
        goto __exit;
    }
#endif

#ifdef BSP_USING_RRH62000_HUMI
    result = rrh62000_register_humidity(name, &cfg, dev);
    if (result != RT_EOK)
    {
        goto __exit;
    }
#endif

#ifdef BSP_USING_RRH62000_TVOC
    result = rrh62000_register_TVOC(name, &cfg, dev);
    if (result != RT_EOK)
    {
        goto __exit;
    }
#endif

#ifdef BSP_USING_RRH62000_DUST
    result = rrh62000_register_Dust(name, &cfg, dev);
    if (result != RT_EOK)
    {
        goto __exit;
    }
#endif

#ifdef BSP_USING_RRH62000_ECO2
    result = rrh62000_register_ECO2(name, &cfg, dev);
    if (result != RT_EOK)
    {
        goto __exit;
    }
#endif

#ifdef BSP_USING_RRH62000_IAQ
    result = rrh62000_register_IAQ(name, &cfg, dev);
    if (result != RT_EOK)
    {
        goto __exit;
    }
#endif

    return RT_EOK;

__exit:
    if(dev)
        rt_free(dev);
    return -RT_ERROR;
}
INIT_DEVICE_EXPORT(rrh62000_device_register);

#ifdef BSP_USING_RRH62000_TEMP
#define RRH_TEMP_DEVICE_NAME "temp_rrh"
static void rrh6200_temp_read(void)
{
    rt_device_t dev = rt_device_find(RRH_TEMP_DEVICE_NAME);
    rt_err_t result;
    rt_uint32_t len;
    struct rt_sensor_data data;

    if(!dev)
    {
        rt_kprintf("No device name %s\n\r", RRH_TEMP_DEVICE_NAME);
        return;
    }
    result = rt_device_open(dev,RT_DEVICE_FLAG_RDONLY);
    if(result != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n\r", RRH_TEMP_DEVICE_NAME);
        return;

    }
    
    len = rt_device_read(dev, 0 ,&data,1);
    if(len)
    {
        rt_kprintf("Temp %d.%d \n\r", data.data.temp >> 16, data.data.temp & 0x0000FFFF);
    }

    result = rt_device_close(dev);
    if(result != RT_EOK)
    {
        rt_kprintf("Close %s Fail\n\r", RRH_TEMP_DEVICE_NAME);
        return;

    }

}
MSH_CMD_EXPORT(rrh6200_temp_read, rrh62000 sample);
#endif


#ifdef BSP_USING_RRH62000_HUMI
#define RRH_HUMI_DEVICE_NAME "humi_rrh"
static void rrh6200_humi_read(void)
{
    rt_device_t dev = rt_device_find(RRH_HUMI_DEVICE_NAME);
    rt_err_t result;
    rt_uint32_t len;
    struct rt_sensor_data data;

    if(!dev)
    {
        rt_kprintf("No device name %s\n\r", RRH_HUMI_DEVICE_NAME);
        return;
    }
    result = rt_device_open(dev,RT_DEVICE_FLAG_RDONLY);
    if(result != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n\r", RRH_HUMI_DEVICE_NAME);
        return;

    }
    
    len = rt_device_read(dev, 0 ,&data,1);
    if(len)
    {
        rt_kprintf("Humidity %d.%d \n\r", data.data.humi >> 16, data.data.humi & 0x0000FFFF);
    }

    result = rt_device_close(dev);
    if(result != RT_EOK)
    {
        rt_kprintf("Close %s Fail\n\r", RRH_HUMI_DEVICE_NAME);
        return;

    }

}
MSH_CMD_EXPORT(rrh6200_humi_read, rrh62000 sample);
#endif

#ifdef BSP_USING_RRH62000_TVOC
#define RRH_TVOC_DEVICE_NAME "tvoc_rrh"
static void rrh6200_tvoc_read(void)
{
    rt_device_t dev = rt_device_find(RRH_TVOC_DEVICE_NAME);
    rt_err_t result;
    rt_uint32_t len;
    struct rt_sensor_data data;

    if(!dev)
    {
        rt_kprintf("No device name %s\n\r", RRH_TVOC_DEVICE_NAME);
        return;
    }
    result = rt_device_open(dev,RT_DEVICE_FLAG_RDONLY);
    if(result != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n\r", RRH_TVOC_DEVICE_NAME);
        return;

    }
    
    len = rt_device_read(dev, 0 ,&data,1);
    if(len)
    {
        rt_kprintf("TVOC %d.%d \n\r", data.data.tvoc >> 16, data.data.tvoc & 0x0000FFFF);
    }

    result = rt_device_close(dev);
    if(result != RT_EOK)
    {
        rt_kprintf("Close %s Fail\n\r", RRH_TVOC_DEVICE_NAME);
        return;

    }

}
MSH_CMD_EXPORT(rrh6200_tvoc_read, rrh62000 sample);
#endif

#ifdef BSP_USING_RRH62000_DUST
#define RRH_DUST_DEVICE_NAME "dust_rrh"
static void rrh6200_dust_read(void)
{
    rt_device_t dev = rt_device_find(RRH_DUST_DEVICE_NAME);
    rt_err_t result;
    rt_uint32_t len;
    struct rt_sensor_data data;

    if(!dev)
    {
        rt_kprintf("No device name %s\n\r", RRH_DUST_DEVICE_NAME);
        return;
    }
    result = rt_device_open(dev,RT_DEVICE_FLAG_RDONLY);
    if(result != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n\r", RRH_DUST_DEVICE_NAME);
        return;

    }
    
    len = rt_device_read(dev, 0 ,&data,1);
    if(len)
    {
        rt_kprintf("dust %d.%d \n\r", data.data.dust >> 16, data.data.dust & 0x0000FFFF);
    }

    result = rt_device_close(dev);
    if(result != RT_EOK)
    {
        rt_kprintf("Close %s Fail\n\r", RRH_DUST_DEVICE_NAME);
        return;

    }

}
MSH_CMD_EXPORT(rrh6200_dust_read, rrh62000 sample);
#endif

#ifdef BSP_USING_RRH62000_ECO2
#define RRH_ECO2_DEVICE_NAME "eco2_rrh"
static void rrh6200_eco2_read(void)
{
    rt_device_t dev = rt_device_find(RRH_ECO2_DEVICE_NAME);
    rt_err_t result;
    rt_uint32_t len;
    struct rt_sensor_data data;

    if(!dev)
    {
        rt_kprintf("No device name %s\n\r", RRH_ECO2_DEVICE_NAME);
        return;
    }
    result = rt_device_open(dev,RT_DEVICE_FLAG_RDONLY);
    if(result != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n\r", RRH_ECO2_DEVICE_NAME);
        return;

    }
    
    len = rt_device_read(dev, 0 ,&data,1);
    if(len)
    {
        rt_kprintf("ECO2 %d.%d \n\r", data.data.eco2 >> 16, data.data.eco2 & 0x0000FFFF);
    }

    result = rt_device_close(dev);
    if(result != RT_EOK)
    {
        rt_kprintf("Close %s Fail\n\r", RRH_ECO2_DEVICE_NAME);
        return;

    }

}
MSH_CMD_EXPORT(rrh6200_eco2_read, rrh62000 sample);
#endif


#ifdef BSP_USING_RRH62000_IAQ
#define RRH_IAQ_DEVICE_NAME "iaq_rrh6"
static void rrh6200_iaq_read(void)
{
    rt_device_t dev = rt_device_find(RRH_IAQ_DEVICE_NAME);
    rt_err_t result;
    rt_uint32_t len;
    struct rt_sensor_data data;

    if(!dev)
    {
        rt_kprintf("No device name %s\n\r", RRH_IAQ_DEVICE_NAME);
        return;
    }
    result = rt_device_open(dev,RT_DEVICE_FLAG_RDONLY);
    if(result != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n\r", RRH_IAQ_DEVICE_NAME);
        return;

    }
    
    len = rt_device_read(dev, 0 ,&data,1);
    if(len)
    {
        rt_kprintf("IAQ %d.%d \n\r", data.data.iaq >> 16, data.data.iaq & 0x0000FFFF);
    }

    result = rt_device_close(dev);
    if(result != RT_EOK)
    {
        rt_kprintf("Close %s Fail\n\r", RRH_IAQ_DEVICE_NAME);
        return;

    }

}
MSH_CMD_EXPORT(rrh6200_iaq_read, rrh62000 sample);
#endif