#include <stdio.h>
#include <unistd.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "hal/gpio_types.h"
#include "portmacro.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "esp_log.h"

#define NUM_PINS 8
#define LPT_START_PIN GPIO_NUM_1
#define LPT_START_PIN_2 GPIO_NUM_10

#define ESP32 // choose which esp that I am currently using

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// If ESP32 C6, define GPIOs
#ifdef ESPC6
const int pins1[] = {1, 2, 3};
const int pins2[] = {1, 2, 3};
#endif

// If standard ESP32-WROOM-32
#ifdef ESP32
const int pins1[] = {GPIO_NUM_15, GPIO_NUM_2, GPIO_NUM_0, GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_5, GPIO_NUM_18};
const int pins2[] = {GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32};
#endif

void set_pins(uint8_t value) {
    /* if (value == 0) { */
    /*     REG_WRITE(GPIO_OUT_W1TC_REG, 0xFF << LPT_START_PIN); */
    /*     return; */
    /* } */

    uint32_t bitmask = 0;
    for (int i = 0; i < ARRAY_SIZE(pins1); i++) {
        if (value & (1 << i)) {
            bitmask |= (1 << (pins1[i]));
            bitmask |= (1 << (pins2[i]));
        }
        printf("%d, ", (uint8_t)(value & (1<<i)));
    }
    printf("\n");
    /* REG_WRITE(GPIO_OUT_W1TS_REG, bitmask); */
}

void lpt_task(void *pvParameters) {
    // setup pins
    for (int i = 0; i < ARRAY_SIZE(pins1); i++) {
        gpio_reset_pin(pins1[i]);
        gpio_reset_pin(pins2[i]);
        gpio_set_direction(pins1[i], GPIO_MODE_OUTPUT);
        gpio_set_direction(pins2[i], GPIO_MODE_OUTPUT);
    }

    ESP_LOGI("DEBUG", "Finished PIN Setup");

    while (1) {
        // ESP_LOGI("DEBUG", "Waiting for Character");
        int command = getc(stdin);
        if (command != EOF) {
            printf("%c: %d\n", command, command);
            set_pins((uint8_t)command);
        }
        vTaskDelay(1);
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI("DEBUG", "started the program!");
    xTaskCreate(lpt_task, "lpt_task", 2048, NULL, 1, NULL);
}
