#include "sensor_iface.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "SENSOR_IFACE";

// Hardware configuration
#define I2C_MASTER_SCL_IO    22    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    21    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM       I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ   100000      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0      /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0      /*!< I2C master doesn't need buffer */

#define BME680_I2C_ADDR      0x76         /*!< BME680 I2C address */
#define BME680_CHIP_ID_REG   0xD0         /*!< BME680 chip ID register */
#define BME680_CHIP_ID       0x61         /*!< BME680 expected chip ID */
#define BME680_TEMP_MSB      0x22         /*!< Temperature MSB register */
#define BME680_TEMP_LSB      0x23         /*!< Temperature LSB register */
#define BME680_TEMP_XLSB     0x24         /*!< Temperature XLSB register */

#define SSD1306_SPI_HOST     HSPI_HOST
#define SSD1306_PIN_CS       5
#define SSD1306_PIN_DC       2
#define SSD1306_PIN_RST      4
#define SSD1306_CMD_DISPLAY_ON  0xAF
#define SSD1306_CMD_SET_MEMORY_MODE 0x20

#define BATTERY_ADC_CHANNEL  ADC1_CHANNEL_0
#define BATTERY_ADC_ATTEN    ADC_ATTEN_DB_11

static bool sensor_initialized = false;
static bool i2c_initialized = false;
static bool spi_initialized = false;
static bool adc_initialized = false;
static spi_device_handle_t ssd1306_spi;

// I2C error checking with timeout and retry
static esp_err_t i2c_master_read_register(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_write_register(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t spi_write_cmd(uint8_t cmd) {
    gpio_set_level(SSD1306_PIN_DC, 0);  // Command mode
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    return spi_device_polling_transmit(ssd1306_spi, &t);
}

bool sensor_iface_init(void) {
    if (sensor_initialized) {
        return true;
    }

    esp_err_t ret;

    // Initialize I2C master
    if (!i2c_initialized) {
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };
        ret = i2c_param_config(I2C_MASTER_NUM, &conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
            return false;
        }
        
        ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                                I2C_MASTER_RX_BUF_DISABLE, 
                                I2C_MASTER_TX_BUF_DISABLE, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
            return false;
        }
        i2c_initialized = true;
        ESP_LOGI(TAG, "I2C master initialized");

        // Verify BME680 connection
        uint8_t chip_id;
        ret = i2c_master_read_register(BME680_I2C_ADDR, BME680_CHIP_ID_REG, &chip_id, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BME680 read chip ID failed: %s", esp_err_to_name(ret));
            return false;
        }
        if (chip_id != BME680_CHIP_ID) {
            ESP_LOGE(TAG, "BME680 chip ID mismatch: expected 0x%02X, got 0x%02X", BME680_CHIP_ID, chip_id);
            return false;
        }
        ESP_LOGI(TAG, "BME680 detected with chip ID: 0x%02X", chip_id);
    }

    // Initialize SPI for SSD1306 OLED
    if (!spi_initialized) {
        spi_bus_config_t buscfg = {
            .miso_io_num = -1,
            .mosi_io_num = 23,
            .sclk_io_num = 18,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4096,
        };
        ret = spi_bus_initialize(SSD1306_SPI_HOST, &buscfg, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
            return false;
        }

        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = 10*1000*1000,  // 10 MHz
            .mode = 0,
            .spics_io_num = SSD1306_PIN_CS,
            .queue_size = 7,
        };
        ret = spi_bus_add_device(SSD1306_SPI_HOST, &devcfg, &ssd1306_spi);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
            return false;
        }
        spi_initialized = true;

        // Configure DC and RST pins
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << SSD1306_PIN_DC) | (1ULL << SSD1306_PIN_RST),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);

        // Reset SSD1306
        gpio_set_level(SSD1306_PIN_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(SSD1306_PIN_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(10));

        // Initialize SSD1306
        spi_write_cmd(SSD1306_CMD_DISPLAY_ON);
        spi_write_cmd(SSD1306_CMD_SET_MEMORY_MODE);
        spi_write_cmd(0x00);  // Horizontal addressing mode
        ESP_LOGI(TAG, "SSD1306 OLED initialized");
    }

    // Initialize ADC for battery monitoring
    if (!adc_initialized) {
        ret = adc1_config_width(ADC_WIDTH_BIT_12);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC width config failed: %s", esp_err_to_name(ret));
            return false;
        }
        
        ret = adc1_config_channel_atten(BATTERY_ADC_CHANNEL, BATTERY_ADC_ATTEN);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
            return false;
        }
        adc_initialized = true;
        ESP_LOGI(TAG, "ADC initialized for battery monitoring");
    }

    sensor_initialized = true;
    ESP_LOGI(TAG, "All sensor interfaces initialized successfully");
    return true;
}

bool bme680_read(bme680_data_t *data) {
    if (!data || !sensor_initialized || !i2c_initialized) {
        return false;
    }

    esp_err_t ret;
    uint8_t temp_data[3];
    
    // Read temperature registers (this is simplified - real BME680 requires calibration)
    ret = i2c_master_read_register(BME680_I2C_ADDR, BME680_TEMP_MSB, temp_data, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BME680 temperature read failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Convert raw ADC to temperature (simplified conversion)
    uint32_t temp_adc = (temp_data[0] << 12) | (temp_data[1] << 4) | (temp_data[2] >> 4);
    data->temperature = (float)temp_adc / 5120.0f;  // Simplified conversion
    
    // For IAQ, we'd need gas sensor data and complex calculations
    // This is a placeholder that would require full BME680 implementation
    data->iaq = 50.0f + (data->temperature - 25.0f) * 0.5f;
    
    ESP_LOGD(TAG, "BME680: temp=%.1f°C, iaq=%.1f (raw_adc=%lu)", 
             data->temperature, data->iaq, temp_adc);
    return true;
}

bool ssd1306_draw_banner(const char *text) {
    if (!text || !sensor_initialized || !spi_initialized) {
        return false;
    }

    // Real SSD1306 implementation would require:
    // 1. Font rendering
    // 2. Framebuffer management
    // 3. Display memory mapping
    // 4. SPI transaction sequences
    
    ESP_LOGI(TAG, "SSD1306 display: %s", text);
    
    // This is a minimal implementation - real one would send pixel data
    spi_write_cmd(0x21);  // Set column address
    spi_write_cmd(0x00);  // Column start
    spi_write_cmd(0x7F);  // Column end
    
    return true;
}

float battery_read_voltage(void) {
    if (!sensor_initialized || !adc_initialized) {
        return -1.0f;
    }

    // Read ADC with multiple samples for better accuracy
    uint32_t adc_sum = 0;
    const int num_samples = 64;
    
    for (int i = 0; i < num_samples; i++) {
        int adc_raw = adc1_get_raw(BATTERY_ADC_CHANNEL);
        if (adc_raw < 0) {
            ESP_LOGE(TAG, "ADC read failed");
            return -1.0f;
        }
        adc_sum += adc_raw;
        vTaskDelay(1);  // Small delay between samples
    }
    
    uint32_t adc_avg = adc_sum / num_samples;
    
    // Convert to voltage (simplified - real implementation needs calibration)
    float voltage = (float)adc_avg * 3.3f / 4095.0f * 2.0f;  // Voltage divider
    
    ESP_LOGD(TAG, "Battery ADC: raw=%lu, voltage=%.2fV", adc_avg, voltage);
    return voltage;
}

static void sensor_task(void *pvParameters) {
    bme680_data_t bme_data;
    float battery_voltage;
    
    ESP_LOGI(TAG, "Sensor monitoring task started");
    
    // Signal that sensors are ready
    printf("SENSOR_OK\n");
    
    while (1) {
        // Read BME680
        if (bme680_read(&bme_data)) {
            printf("BME680: Temperature=%.1f°C, IAQ=%.1f\n", 
                   bme_data.temperature, bme_data.iaq);
        } else {
            printf("BME680: Read failed\n");
        }
        
        // Read battery voltage
        battery_voltage = battery_read_voltage();
        if (battery_voltage > 0) {
            printf("Battery: %.2fV\n", battery_voltage);
        } else {
            printf("Battery: Read failed\n");
        }
        
        // Update OLED display
        char display_text[32];
        snprintf(display_text, sizeof(display_text), "T:%.1f IAQ:%.0f", 
                 bme_data.temperature, bme_data.iaq);
        ssd1306_draw_banner(display_text);
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
    }
}

void sensor_task_start(void) {
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
} 