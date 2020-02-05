#include "dht22.hpp"
#include "ssd1306.hpp"

#include <string.h>
#include <freertos/FreeRTOS.h>
#include "freertos/queue.h"
#include <freertos/task.h>
#include <nvs_flash.h>
#include <esp_log.h>

typedef struct {
    float temperature;
    float humidity;
} Measurement;

static const char T_DHT[] = "dht22";
static const char T_SSD[] = "screen";
static xQueueHandle event_queue;
static gpio_num_t DHT_GPIO = (gpio_num_t)CONFIG_DHT22_SENSOR_GPIO;

static void Display_task(void *pvParameter) {
    ESP_LOGI(T_SSD, "starting SSD1306 Task");

    SSD1306 display = SSD1306((gpio_num_t)4, (gpio_num_t)5);
    display.clear();
    display.put_line(0, 1, "Hum   %");
    display.put_line(0, 4, "Temp ^C");

    Measurement measurement;
    char buffer[9];
    while (true) {
        if (xQueueReceive(event_queue, &measurement, portMAX_DELAY)) {

            memset(buffer, 0, 9);
            sniprintf(buffer, 8, "%.1f", measurement.humidity);
            display.put_line(1, 2, buffer);

            memset(buffer, 0, 9);
            sniprintf(buffer, 8, "%.1f", measurement.temperature);
            display.put_line(1, 5, buffer);

            display.flush();
            ESP_LOGI(T_SSD, "hum: %.1f%% tmp: %.1f `C", measurement.humidity, measurement.temperature);
        }
    }
}

static void DHT_task(void *pvParameter) {
    ESP_LOGI(T_DHT, "starting DHT Task");
    DHT22 dht = DHT22(DHT_GPIO);
    vTaskDelay(7000 / portTICK_RATE_MS);

    Measurement measurement;
    while (true) {
        ESP_LOGD(T_DHT, "Reading...");
        // you must not call this faster than 1 call in 2 seconds (0.5 Hz)
        int err = dht.readDHT();
        if (err != DHT_OK) {
            switch (err) {
                case DHT_TIMEOUT_ERROR:
#ifdef CONFIG_DHT22_TELEMETRY
                    dht.printTelemetry();
#endif
                    ESP_LOGE(T_DHT, "sensor timeout error\n");
                    break;
                case DHT_CHECKSUM_ERROR:
                    ESP_LOGE(T_DHT, "crc error\n");
                    break;
                default:
                    ESP_LOGE(T_DHT, "unknown error: %i\n", err);
            }
        } else {
            measurement.humidity = dht.getHumidity();
            measurement.temperature = dht.getTemperature();
            xQueueSend(event_queue, &measurement, portMAX_DELAY);
            ESP_LOGD(T_DHT, "hum: %.1f%% tmp: %.1f C", dht.getHumidity(), dht.getTemperature());
        }
        vTaskDelay(10000 / portTICK_RATE_MS);
    }
}

extern "C" void app_main(void) {
    // initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    event_queue = xQueueCreate(32, sizeof(Measurement));

    xTaskCreate(&Display_task, "Display_task", 2048, NULL, 10, NULL);
    xTaskCreate(&DHT_task, "DHT_task", 2048, NULL, 8, NULL);
}
