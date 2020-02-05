#include "dht22.hpp"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>

static char TAG[] = "DHT22";


DHT22::DHT22(gpio_num_t gpio) : m_gpio(gpio), humidity(0.0), temperature(0.0) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (uint32_t) 0x1 << gpio;
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(gpio, 1));
}

int IRAM_ATTR DHT22::waitBusLevel(int64_t usec, int state) {
    if (gpio_get_level(m_gpio) == state) {
        return 0;
    }

    int64_t start_us = esp_timer_get_time();
    int64_t now_us = start_us;
    int64_t timeout = start_us + usec;

    while(now_us < timeout) {
        ets_delay_us(1);
        if (gpio_get_level(m_gpio) == state) {
            now_us = esp_timer_get_time();
            return (int)(now_us - start_us);
        } else {
            now_us = esp_timer_get_time();
        }
    }
    return -1;
}

int DHT22::readDHT() {
    uint8_t byte;
    int usec;
    int bit_n = 0;

#ifdef CONFIG_DHT22_TELEMETRY
    // reset telemetry array
    for (int i = 0; i < 80; i++) {
        telemetry[i] = -1;
    }
#endif
                                                                       // min    typ    max
    // send start signal
    gpio_set_level(m_gpio, 0);
    ets_delay_us(1000);                                            // 0.8    1      20    ms
    gpio_set_level(m_gpio, 1);

    // wait for DHT22 report it is ready
    if (waitBusLevel(200, 0) == -1) {                      // 20     30     200   us
        ESP_LOGE(TAG, "doesn't not respond to START");
        return DHT_TIMEOUT_ERROR;
    }

    int err = DHT_OK;
    portENTER_CRITICAL();
    // wait for DHT22 release the bus
    if (waitBusLevel(85, 1) == -1) {                       // 75     80     85    us
        ESP_LOGE(TAG, "doesn't reset bus in time after ACK");
        return DHT_TIMEOUT_ERROR;
    }
    // skip pre-data gap
    if (waitBusLevel(85, 0) == -1) {                       // 75     80     85    us
        ESP_LOGE(TAG, "doesn't not start data sending in time");
        return DHT_TIMEOUT_ERROR;
    }
    // read data bits
    for (int i = 0; i < 5; i++) {
        byte = 0;
        for (int b = 0; b < 8; b++) {
            // wait bit
            usec = waitBusLevel(65, 1);                    // 48     50     55    us
#ifdef CONFIG_DHT22_TELEMETRY
            telemetry[bit_n] = usec;
#endif
            if (usec == -1) {
                err = DHT_TIMEOUT_ERROR;
            }
            bit_n++;
            // read bit                                                // 22     26     30    us        bit 0
            usec = waitBusLevel(75, 0);                    // 68     70     75    us        bit 1
#ifdef CONFIG_DHT22_TELEMETRY
            telemetry[bit_n] = usec;
#endif
            if (usec == -1) {
                err = DHT_TIMEOUT_ERROR;
            }
            if (err == DHT_OK) {
                byte <<= 1;
                if (usec > 49) byte |= 0x1;
            }
            bit_n++;
        }
        data[i] = byte;
    }
    // wait release
    if (waitBusLevel(56, 1) == -1) {                       // 45     50     55    us
        ESP_LOGE(TAG, "doesn't released bus in time after DATA");
        return DHT_TIMEOUT_ERROR;
    }
    portEXIT_CRITICAL();

    if (err != DHT_OK) {
        return err;
    }

    // check CRC
    uint16_t sum = 0;
    for (int i = 0; i < 4; i++) {
        sum += (uint16_t)data[i];
    }
    if (((data[0] + data[1] + data[2] + data[3]) & 0xff) != data[4]) return DHT_CHECKSUM_ERROR;

    // calculate data values
    uint16_t hum = ((uint16_t)data[0] << 8) + data[1];
    humidity = (float)(hum / 10.0);

    int16_t temp = ((uint16_t)(data[2] & 0x7f) << 8) + data[3];
    if (data[2] >> 7) {
        temp *= -1;
    }
    temperature = (float)(temp / 10.0);
    return DHT_OK;
}

#ifdef CONFIG_DHT22_TELEMETRY
void DHT22::printTelemetry() {
    ESP_LOGE(TAG, "Data Telemetry:");
    for (int i = 0; i < 80; i += 2) {
        ESP_LOGE(TAG, "    %02i bit: low %02i usec, high %02i usec", i / 2, telemetry[i], telemetry[i + 1]);
    }
}
#endif
