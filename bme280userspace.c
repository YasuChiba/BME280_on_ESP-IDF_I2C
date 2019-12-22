
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"

#include "bme280.h"

#define SDA_PIN GPIO_NUM_25
#define SCL_PIN GPIO_NUM_21

esp_err_t i2c_master_init();
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void print_sensor_data(struct bme280_data *comp_data);
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev);

esp_err_t i2c_master_init()
{
  i2c_config_t i2c_config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = SDA_PIN,
      .scl_io_num = SCL_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 100000};
  i2c_param_config(I2C_NUM_0, &i2c_config);
  return i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{

  /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

  /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

  int8_t rslt = 0;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_READ, true);

  if (len > 1)
  {
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
  if (espRc == ESP_OK)
  {
    rslt = 0;
  }
  else
  {
    rslt = -1;
  }

  i2c_cmd_link_delete(cmd);
  return rslt;
}

void user_delay_ms(uint32_t period)
{
  vTaskDelay(period / portTICK_PERIOD_MS);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

  /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
  int8_t rslt = 0;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_write(cmd, data, len, true);
  i2c_master_stop(cmd);

  esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
  if (espRc == ESP_OK)
  {
    rslt = 0;
  }
  else
  {
    rslt = -1;
  }

  i2c_cmd_link_delete(cmd);
  return rslt;
}

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE

  configPRINTF(("%0.2f, %0.2f, %0.2f\r\n", comp_data->temperature, comp_data->pressure, comp_data->humidity));
#else
  configPRINTF(("%ld, %ld, %ld\r\n", comp_data->temperature, comp_data->pressure, comp_data->humidity));
#endif
}

int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
  int8_t rslt;
  uint8_t settings_sel;
  struct bme280_data comp_data;

  /* Recommended mode of operation: Indoor navigation */
  dev->settings.osr_h = BME280_OVERSAMPLING_1X;
  dev->settings.osr_p = BME280_OVERSAMPLING_16X;
  dev->settings.osr_t = BME280_OVERSAMPLING_2X;
  dev->settings.filter = BME280_FILTER_COEFF_16;

  settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

  rslt = bme280_set_sensor_settings(settings_sel, dev);
  if (rslt != BME280_OK)
  {
    configPRINTF(("set sensor fail\n"));
    return rslt;
  }
  else
  {
    configPRINTF(("set sensor success\n"));
  }

  configPRINTF(("Temperature, Pressure, Humidity\r\n"));
  /* Continuously stream sensor data */
  while (1)
  {
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);

    /* Wait for the measurement to complete and print data @25Hz */
    dev->delay_ms(40);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);

    print_sensor_data(&comp_data);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  return rslt;
}
