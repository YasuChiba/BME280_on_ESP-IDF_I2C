# BME280_on_ESP-IDF_I2C


Using BME280 on ESP-IDF(ESP32) using I2C and official driver(https://github.com/BoschSensortec/BME280_driver).
```

#include "bme280userspace.c"
#include "bme280.c"

int app_main(void)
{
    prvMiscInitialization();

    if (SYSTEM_Init() == pdPASS)
    {
        i2c_master_init();
        struct bme280_dev dev;
        int8_t rslt = BME280_OK;
        dev.dev_id = BME280_I2C_ADDR_PRIM;
        dev.intf = BME280_I2C_INTF;
        dev.read = user_i2c_read;
        dev.write = user_i2c_write;
        dev.delay_ms = user_delay_ms;

        rslt = bme280_init(&dev);
        if (rslt != BME280_OK)
        {
            configPRINTF(("fail init\n "));
        }

        stream_sensor_data_forced_mode(&dev);
    }

    return 0;
}
```


See also 
https://www.mgo-tec.com/blog-entry-esp32-bme280-sensor-library.html
