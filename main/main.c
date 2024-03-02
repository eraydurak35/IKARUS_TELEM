
#include "freertos/FreeRTOS.h"
#include "comminication.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "string.h"
#include <stdio.h>
#include "uart.h"


static TaskHandle_t task1_handler;
static TaskHandle_t task2_handler;

void IRAM_ATTR timer1_callback(void *arg)
{
    xTaskNotifyFromISR(task1_handler, 1, eIncrement, false);
}

void task_1(void *pvParameters)
{
    static uint32_t notification;

    while (1)
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &notification, portMAX_DELAY) == pdTRUE)
        {
            send_telem_to_pc();
        }
    }
}


void task_2(void *pvParameters)
{
    static uart_data_t recv_data;
    while (1)
    {
        uart_read(UART_NUM_0, &recv_data, 20);
        parse_pc_data(&recv_data);
    }

}

void app_main(void)
{
    comminication_init();

    esp_timer_handle_t timer1;
    const esp_timer_create_args_t timer1_args =
    {
        .callback = &timer1_callback,
        .arg = NULL,
        .name = "timer1"
    };

    esp_timer_create(&timer1_args, &timer1);
    esp_timer_start_periodic(timer1, 100000); //10Hz

    xTaskCreatePinnedToCore(&task_1, "task1", 1024 * 4, NULL, 1, &task1_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_2, "task2", 1024 * 4, NULL, 1, &task2_handler, tskNO_AFFINITY);
}

