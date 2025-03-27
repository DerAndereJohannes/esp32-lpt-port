#include <stdio.h>
#include <unistd.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "portmacro.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "esp_log.h"
#include "driver/uart.h"

#define PIN_RESET_TIME_MS pdMS_TO_TICKS(100) // Time in milliseconds 
#define DEBUG // Comment out to get production version without debug information
#define ESP32 // Choose which esp that is currently being used

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0])) // Function to get size of an array
#define QUEUE_LENGTH 20
#define QUEUE_SIZE sizeof(uint8_t)
#define UART_PORT UART_NUM_0
#define UART_BUFFER_SIZE 1024

QueueHandle_t triggerQueue;

// If ESP32 C6, define GPIOs
#ifdef ESPC6
const int pins1[] = {1, 2, 3};
const int pins2[] = {1, 2, 3};
#endif

// If standard ESP32-WROOM-32
#ifdef ESP32
/* const int pins1[] = {GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_4, GPIO_NUM_0, GPIO_NUM_2}; */
/* const int pins2[] = {GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13}; */

// 6 pins
/* const int pins1[] = {GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_4}; */
/* const int pins2[] = {GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13}; */

// 5 pins
const int pins1[] = {GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_17, GPIO_NUM_16};
/* /1* const int pins1[] = {GPIO_NUM_5, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_4, GPIO_NUM_0, GPIO_NUM_2}; *1/ */
const int pins2[] = {GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12};
#endif

// Flag to check for setup end
static volatile bool setup_done = false;
/* static uint32_t last_bitmask = 0; */

void set_pins(uint8_t value) {
    /* REG_WRITE(GPIO_OUT_W1TC_REG, last_bitmask); */
    /* vTaskDelay(PIN_RESET_TIME_MS); */

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
    /* last_bitmask = bitmask; */
}

void lpt_task(void *pvParameters) {
    // Wait for setup to finish
    while (!setup_done) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Guarantee that the setup function returns
    vTaskDelay(pdMS_TO_TICKS(500));

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


void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,                      // Set baud rate
        .data_bits = UART_DATA_8_BITS,            // 8 data bits
        .parity    = UART_PARITY_DISABLE,         // No parity
        .stop_bits = UART_STOP_BITS_1,            // 1 stop bit
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE     // No flow control
    };
    
    // Apply configuration
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));

    // Set UART0 pins (RX=3, TX=1 by default on ESP32)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install the UART driver with RX & TX buffers
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUFFER_SIZE, 0, 0, NULL, 0));
}

void input_task(void *pvParameters) {
    /* int trigger; */
    uint8_t u8_trigger;

    while (!setup_done) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Guarantee that the setup function returns
    vTaskDelay(pdMS_TO_TICKS(500));

    uart_init();

    while(1) {
        int read = uart_read_bytes(UART_NUM_0, &u8_trigger, 1, 1000 / portTICK_PERIOD_MS);
        if (read > 0) {
            #ifdef DEBUG
            printf("%c: %d", u8_trigger, u8_trigger);
            printf("Received Byte Normal: (0x%02X)\n", u8_trigger);
            #endif
            xQueueSend(triggerQueue, &u8_trigger, portMAX_DELAY);
        }   
        vTaskDelay(1);
        /* trigger = getc(stdin); */
        /* if (trigger != EOF) { */
        /*     u8_trigger = (uint8_t)trigger; */
        /*     #ifdef DEBUG */
        /*     printf("%c: %d, u8 = %c: %d\n", trigger, trigger, u8_trigger, u8_trigger); */
        /*     printf("Received Byte Normal: (0x%02X), U8: (0x%02X)\n", trigger, u8_trigger); */
        /*     #endif */
        /*     xQueueSend(triggerQueue, &u8_trigger, portMAX_DELAY); */
        /* } */   
        /* vTaskDelay(1); */
    }
}

void output_task(void *pvParameters) {
    uint8_t trigger;

    while (!setup_done) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Guarantee that the setup function returns
    vTaskDelay(pdMS_TO_TICKS(500));

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

    while(1) {
        if (xQueueReceive(triggerQueue, &trigger, portMAX_DELAY)) {
            set_pins(trigger);
        }
        vTaskDelay(1);
    }
}

void app_main(void)
{
    triggerQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_SIZE);

    #ifdef DEBUG
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI("DEBUG", "started the program!");
    #endif

    /* xTaskCreate(lpt_task, "lpt_task", 2048, NULL, 1, NULL); */
    xTaskCreatePinnedToCore(input_task, "trigger_input", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(output_task, "trigger_output", 4096, NULL, 5, NULL, 1);
    setup_done = true;
}
