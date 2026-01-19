#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hello_lvgl.h"

static const char *TAG = "app";

void app_main(void) {
    ESP_LOGI(TAG, "Boot: init LCD + Touch + LVGL...");
    hello_lvgl_start();

    // Nada mais a fazer aqui: a task do LVGL fica rodando.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
