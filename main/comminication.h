#ifndef COMM_H
#define COMM_H

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "uart.h"

#define CHANNEL 6
#define DATARATE WIFI_PHY_RATE_54M//WIFI_PHY_RATE_24M

#define MSG1_HEADER 0xFF
#define MSG2_HEADER 0xFE
#define MSG3_HEADER 0xFD

void comminication_init();
void send_telem_to_pc();
void parse_pc_data(uart_data_t *recv);

#endif