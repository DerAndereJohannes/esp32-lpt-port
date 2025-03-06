#include <stdio.h>
#include <unistd.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "hal/gpio_types.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "esp_log.h"

#define PIN_RESET_TIME_MS 10 // Time in milliseconds 
// #define DEBUG // Comment out to get production version without debug information
#define ESP32 // Choose which esp that is currently being used

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0])) // Function to get size of an array

// If ESP32 C6, define GPIOs
#ifdef ESPC6
const int pins1[] = {1, 2, 3};
const int pins2[] = {1, 2, 3};
#endif

// If standard ESP32-WROOM-32
#ifdef ESP32
const int pins1[] = {GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_4, GPIO_NUM_0, GPIO_NUM_2};
const int pins2[] = {GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13};
#endif

void set_pins(uint8_t value) {

    uint32_t bitmask = 0;
    for (int i = 0; i < ARRAY_SIZE(pins1); i++) {
        if (value & (1 << i)) {
            bitmask |= (1 << (pins1[i]));
            bitmask |= (1 << (pins2[i]));
        }
        #ifdef DEBUG
        printf("%d, ", (uint8_t)(value & (1<<i)));
        #endif
    }
    #ifdef DEBUG
    printf("\n");
    #endif
    REG_WRITE(GPIO_OUT_W1TS_REG, bitmask);
    vTaskDelay(PIN_RESET_TIME_MS);
    REG_WRITE(GPIO_OUT_W1TC_REG, bitmask);
}

void lpt_task(void *pvParameters) {
    // setup pins
    for (int i = 0; i < ARRAY_SIZE(pins1); i++) {
        gpio_reset_pin(pins1[i]);
        gpio_reset_pin(pins2[i]);
        gpio_set_direction(pins1[i], GPIO_MODE_OUTPUT);
        gpio_set_direction(pins2[i], GPIO_MODE_OUTPUT);
    }

    #ifdef DEBUG
    ESP_LOGI("DEBUG", "Finished PIN Setup");
    #endif

    while (1) {
        int command = getc(stdin);
        if (command != EOF) {
            #ifdef DEBUG
            printf("%c: %d\n", command, command);
            #endif
            set_pins((uint8_t)command);
        }
        vTaskDelay(1);
    }
}

void app_main(void)
{
    #ifdef DEBUG
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI("DEBUG", "started the program!");
    #endif
    xTaskCreate(lpt_task, "lpt_task", 2048, NULL, 1, NULL);
}
