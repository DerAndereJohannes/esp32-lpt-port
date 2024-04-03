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

void set_pins(uint8_t value) {
    /* if (value == 0) { */
    /*     REG_WRITE(GPIO_OUT_W1TC_REG, 0xFF << LPT_START_PIN); */
    /*     return; */
    /* } */

    uint32_t bitmask = 0;
    for (int i = 0; i < NUM_PINS; i++) {
        if (value & (1 << i)) {
            bitmask |= (1 << (LPT_START_PIN + i));
        }
        printf("%d, ", (uint8_t)(value & (1<<i)));
    }
    printf("\n");
    /* REG_WRITE(GPIO_OUT_W1TS_REG, bitmask); */
}

void lpt_task(void *pvParameters) {
    // setup pins
    for (int i = 0; i < NUM_PINS; i++) {
        gpio_reset_pin(LPT_START_PIN + i);
        gpio_set_direction(LPT_START_PIN + i, GPIO_MODE_OUTPUT);
    }

    while (1) {
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
