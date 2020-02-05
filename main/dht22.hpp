/*
 *  DHT22 temperature/humidity sensor driver
 */
#pragma once

#include <driver/gpio.h>

#define DHT_OK 0
#define DHT_CHECKSUM_ERROR -1
#define DHT_TIMEOUT_ERROR -2

class DHT22 {
public:
    explicit DHT22(gpio_num_t gpio);

    int readDHT();
    float getHumidity() { return humidity; };
    float getTemperature() { return temperature; };

#ifdef CONFIG_DHT22_TELEMETRY
    void printTelemetry();
#endif

private:
    gpio_num_t m_gpio;
    float humidity;
    float temperature;

    uint8_t data[5];  // 40 bits

#ifdef CONFIG_DHT22_TELEMETRY
    int telemetry[80];  // lengths of start_bit and bit_0_or1 pulses
#endif

    int waitBusLevel(int64_t usec, int state);
};
