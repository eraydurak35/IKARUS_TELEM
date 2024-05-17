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
#define MSG4_HEADER 0xFC
#define MSG5_HEADER 0xFB
#define MSG6_HEADER 0xFA
#define MSG7_HEADER 0xF9

void comminication_init();
void send_telem_to_pc();
void parse_pc_data(uart_data_t *recv);
void calculate_packet_drop_ratio();

#endif