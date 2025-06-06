#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sensor_iface.h"

static const char *TAG = "OLYMPUS_MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "Project Olympus - Digital Twin Node Starting");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    
    // Initialize sensor interface
    if (!sensor_iface_init()) {
        ESP_LOGE(TAG, "Failed to initialize sensor interface");
        return;
    }
    
    ESP_LOGI(TAG, "Sensor interface initialized successfully");
    
    // Start sensor monitoring task
    sensor_task_start();
    
    ESP_LOGI(TAG, "Sensor monitoring task started");
    ESP_LOGI(TAG, "System ready - monitoring sensors every 1 second");
    
    // Main loop - keep the application running
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10 second delay
        ESP_LOGI(TAG, "System heartbeat - uptime: %lu seconds", 
                 (unsigned long)(esp_timer_get_time() / 1000000));
    }
} 