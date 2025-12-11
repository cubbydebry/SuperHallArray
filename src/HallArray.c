// HallArray.c 
// Cubby DeBry 2025

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#define BUFFER_SIZE 16

#define SPI_PORT spi0
#define PIN_MISO 0
#define PIN_SCK 2
#define PIN_MOSI 3

#define CS1 4
#define CS2 5
#define CS3 6
#define CS4 7
#define CS5 8

#define IN0 (0x7 << 3)
#define IN1 (0x6 << 3)
#define IN2 (0x5 << 3)
#define IN3 (0x0 << 3)
#define IN4 (0x1 << 3)
#define IN5 (0x2 << 3)
#define IN6 (0x3 << 3)
#define IN7 (0x4 << 3)

static uint adc_buffer[BUFFER_SIZE];
static const uint8_t input_ctrl[8] = {IN0, IN1, IN2, IN3, IN4, IN5, IN6, IN7};
static const uint8_t channel_ctrl[5] = {CS1, CS2, CS3, CS4, CS5};

void spi_init_adc_bus(void){
    spi_init(SPI_PORT, .1 * 1000 * 1000); // 1 MHz
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    for (int i = 0; i < 5; i++){
        gpio_init(channel_ctrl[i]);
        gpio_set_dir(channel_ctrl[i], GPIO_OUT);
        gpio_put(channel_ctrl[i], 1); // Inactive (Active Low)
    }
}

void adc_init(void){

}

uint16_t spi_transfer16(uint16_t tx){
    uint8_t tx_buf[4] = { (tx >> 8) & 0xFF, tx & 0xFF, 0x00, 0x00};
    uint8_t rx_buf[4];
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, 2);
    return ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
}

uint16_t adc_read(uint8_t channel, uint8_t input){
    gpio_put(channel, 0);
    spi_transfer16((uint16_t)input << 8);
    uint16_t raw = spi_transfer16((uint16_t)input << 8);
    gpio_put(channel, 1);
    return raw;
}

void main_task(__unused void *params) {
    printf("Entering main task\n");
    while(1) {
        for(int i = 0; i < 8; i++){
            for(int j = 0; j < 10000; j++){
                uint16_t raw = adc_read(CS1, input_ctrl[i]);
                printf("Channel %d ADC Reading: %u\n", i, raw);
            }
        }
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(void)
{
    stdio_init_all();
    cyw43_arch_init();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    spi_init_adc_bus();
    TaskHandle_t readtask;
    xTaskCreate(main_task, "Thread",
                configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &readtask);
    printf("Starting scheduler\n");
    vTaskStartScheduler();
    return 0;
}
