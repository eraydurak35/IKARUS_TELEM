#include <string.h>
#include "comminication.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "nvs_flash.h"
#include "uart.h"
#include "driver/gpio.h"


typedef struct {
    uint16_t frame_head;
    uint16_t duration;
    uint8_t destination_address[6];
    uint8_t source_address[6];
    uint8_t broadcast_address[6];
    uint16_t sequence_control;

    uint8_t category_code;
    uint8_t organization_identifier[3]; // 0x18fe34
    uint8_t random_values[4];
    struct {
        uint8_t element_id;                 // 0xdd
        uint8_t lenght;                     //
        uint8_t organization_identifier[3]; // 0x18fe34
        uint8_t type;                       // 4
        uint8_t version;
        uint8_t body[0];
    } vendor_specific_content;
} __attribute__ ((packed)) espnow_frame_format_t;


static const uint8_t drone_mac_address[6] = {0x04, 0x61, 0x05, 0x05, 0x3A, 0xE4};
static esp_now_peer_info_t peerInfo;

static uint8_t telemetry_size = 0;
static uint8_t send_to_pc_buffer[255];
static uint8_t send_to_drone_buffer[255];
static uint8_t RSSI = 0;
static uint8_t is_new_packet_received = 0;
static uint8_t packet_drop_ratio = 0;

static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static void checksum_generate(uint8_t *data, uint8_t size, uint8_t *cs1, uint8_t *cs2);
static uint8_t checksum_verify(uint8_t *data, uint8_t size);

void comminication_init()
{
    nvs_flash_init();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Set Wi-Fi protocol to long range mode
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));

    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());

    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_receive_cb));
    memcpy(peerInfo.peer_addr, drone_mac_address, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 0);
    uart_begin(UART_NUM_0, 921600, 1, 3, UART_PARITY_DISABLE);
}

void parse_pc_data(uart_data_t *recv)
{

    static uint8_t msg1_header_found;
    static uint8_t msg2_header_found;
    static uint8_t msg3_header_found;
    static uint8_t msg4_header_found;
    static uint8_t msg5_header_found;
    static uint8_t msg6_header_found;
    static uint8_t msg7_header_found;
    static uint8_t size = 0;
    static uint8_t is_size_determined = 0;
    static uint8_t counter = 1;

    for (uint16_t i = 0; i < recv->lenght; i++)
    {
        if (msg1_header_found == 1)
        {
            if (is_size_determined == 0)
            {
                size = recv->data[i];
                is_size_determined = 1;
                send_to_drone_buffer[0] = 0xff;
            }
            else
            {
                send_to_drone_buffer[counter++] = recv->data[i];

                if (counter == size+1)
                {
                    if (checksum_verify(send_to_drone_buffer+1, size) == 1)
                    {
                        esp_now_send(drone_mac_address, send_to_drone_buffer, size - 2);
                    }
                    counter = 1;
                    msg1_header_found = 0;
                    is_size_determined = 0;
                }
            }
        }
        else if (msg2_header_found == 1)
        {
            if (is_size_determined == 0)
            {
                size = recv->data[i];
                is_size_determined = 1;
                send_to_drone_buffer[0] = 0xfe;
            }
            else
            {
                send_to_drone_buffer[counter++] = recv->data[i];

                if (counter == size+1)
                {
                    if (checksum_verify(send_to_drone_buffer+1, size) == 1)
                    {
                        esp_now_send(drone_mac_address, send_to_drone_buffer, size - 2);
                    }
                    counter = 1;
                    msg2_header_found = 0;
                    is_size_determined = 0;
                }
            }
        }
        else if (msg3_header_found == 1)
        {
            if (is_size_determined == 0)
            {
                size = recv->data[i];
                is_size_determined = 1;
                send_to_drone_buffer[0] = 0xfd;
            }
            else
            {
                send_to_drone_buffer[counter++] = recv->data[i];

                if (counter == size+1)
                {
                    if (checksum_verify(send_to_drone_buffer+1, size) == 1)
                    {
                        esp_now_send(drone_mac_address, send_to_drone_buffer, size - 2);
                    }
                    counter = 1;
                    msg3_header_found = 0;
                    is_size_determined = 0;
                }
            }
        }
        else if (msg4_header_found == 1)
        {
            if (is_size_determined == 0)
            {
                size = recv->data[i];
                is_size_determined = 1;
                send_to_drone_buffer[0] = 0xfc;
            }
            else
            {
                send_to_drone_buffer[counter++] = recv->data[i];

                if (counter == size+1)
                {
                    if (checksum_verify(send_to_drone_buffer+1, size) == 1)
                    {
                        esp_now_send(drone_mac_address, send_to_drone_buffer, size - 2);
                    }
                    counter = 1;
                    msg4_header_found = 0;
                    is_size_determined = 0;
                }
            }
        }
        else if (msg5_header_found == 1)
        {
            if (is_size_determined == 0)
            {
                size = recv->data[i];
                is_size_determined = 1;
                send_to_drone_buffer[0] = 0xfb;
            }
            else
            {
                send_to_drone_buffer[counter++] = recv->data[i];

                if (counter == size+1)
                {
                    if (checksum_verify(send_to_drone_buffer+1, size) == 1)
                    {
                        esp_now_send(drone_mac_address, send_to_drone_buffer, size - 2);
                    }
                    counter = 1;
                    msg5_header_found = 0;
                    is_size_determined = 0;
                }
            }
        }
        else if (msg6_header_found == 1)
        {
            if (is_size_determined == 0)
            {
                size = recv->data[i];
                is_size_determined = 1;
                send_to_drone_buffer[0] = 0xfa;
            }
            else
            {
                send_to_drone_buffer[counter++] = recv->data[i];

                if (counter == size+1)
                {
                    if (checksum_verify(send_to_drone_buffer+1, size) == 1)
                    {
                        esp_now_send(drone_mac_address, send_to_drone_buffer, size - 2);
                    }
                    counter = 1;
                    msg6_header_found = 0;
                    is_size_determined = 0;
                }
            }
        }
        else if (msg7_header_found == 1)
        {
            if (is_size_determined == 0)
            {
                size = recv->data[i];
                is_size_determined = 1;
                send_to_drone_buffer[0] = 0xf9;
            }
            else
            {
                send_to_drone_buffer[counter++] = recv->data[i];

                if (counter == size+1)
                {
                    if (checksum_verify(send_to_drone_buffer+1, size) == 1)
                    {
                        esp_now_send(drone_mac_address, send_to_drone_buffer, size - 2);
                    }
                    counter = 1;
                    msg7_header_found = 0;
                    is_size_determined = 0;
                }
            }
        }

        else if (recv->data[i] == MSG1_HEADER)
        {
            msg1_header_found = 1;
        }
        else if (recv->data[i] == MSG2_HEADER)
        {
            msg2_header_found = 1;
        }
        else if (recv->data[i] == MSG3_HEADER)
        {
            msg3_header_found = 1;
        }
        else if (recv->data[i] == MSG4_HEADER)
        {
            msg4_header_found = 1;
        }
        else if (recv->data[i] == MSG5_HEADER)
        {
            msg5_header_found = 1;
        }
        else if (recv->data[i] == MSG6_HEADER)
        {
            msg6_header_found = 1;
        }
        else if (recv->data[i] == MSG7_HEADER)
        {
            msg7_header_found = 1;
        }
    }
}

void send_telem_to_pc()
{
    if (telemetry_size > 0)
    {
        send_to_pc_buffer[0] = 0xFF;
        send_to_pc_buffer[1] = telemetry_size + 5;
        send_to_pc_buffer[2] = RSSI;
        send_to_pc_buffer[3] = packet_drop_ratio;

        uint8_t cs1, cs2;
        checksum_generate(send_to_pc_buffer + 2, telemetry_size + 2, &cs1, &cs2);

        send_to_pc_buffer[telemetry_size + 7 - 3] = cs1;
        send_to_pc_buffer[telemetry_size + 7 - 2] = cs2;
        send_to_pc_buffer[telemetry_size + 7 - 1] = 0x69;
        uart_write(UART_NUM_0, send_to_pc_buffer, telemetry_size + 7);
    }
}

void calculate_packet_drop_ratio() // 10Hz
{
    static uint8_t call_count = 0; // Counter to track the number of times the function is called in a second
    static uint8_t missed_packet_count = 0;

    if (!is_new_packet_received) missed_packet_count++;
    else is_new_packet_received = 0;
    
    call_count++;
    
    if (call_count >= 20) 
    {
        // Reset counters for the next second
        packet_drop_ratio = missed_packet_count * 5;
        missed_packet_count = 0;
        call_count = 0;
    }
}

static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    is_new_packet_received = 1;

    wifi_promiscuous_pkt_t* promiscuous_pkt = (wifi_promiscuous_pkt_t*)(data - sizeof (wifi_pkt_rx_ctrl_t) - sizeof (espnow_frame_format_t));
    wifi_pkt_rx_ctrl_t* rx_ctrl = &promiscuous_pkt->rx_ctrl;

    if (rx_ctrl->rssi > 0) RSSI = 0;
    else
    {
        float unf_rssi = rx_ctrl->rssi * -1;
        static float filt_rssi = 0;
        filt_rssi += (unf_rssi - filt_rssi) * 0.4f;
        RSSI = (uint8_t)(filt_rssi);
    } 

    if (data[0] == 0xFF)
    {
        memcpy(send_to_pc_buffer + 4, data + 1, len - 1);
        telemetry_size = len - 1;
    }

    else if (data[0] == 0xFE)
    {
        uint8_t config_buffer[len + 1 + 3];
        config_buffer[0] = 0xFE;
        config_buffer[1] = len - 1 + 3;
        memcpy(config_buffer + 2, data + 1, len - 1);
        uint8_t cs1, cs2;
        checksum_generate(config_buffer + 2, len - 1, &cs1, &cs2);
        config_buffer[len + 1 + 0] = cs1;
        config_buffer[len + 1 + 1] = cs2;
        config_buffer[len + 1 + 2] = 0x69;
        uart_write(UART_NUM_0, config_buffer, sizeof(config_buffer));
    }

    else if (data[0] == 0xFD)
    {
        uint8_t wp_buffer[len + 1 + 3];
        wp_buffer[0] = 0xFD;
        wp_buffer[1] = len - 1 + 3;
        memcpy(wp_buffer + 2, data + 1, len - 1);
        uint8_t cs1, cs2;
        checksum_generate(wp_buffer + 2, len - 1, &cs1, &cs2);
        wp_buffer[len + 1 + 0] = cs1;
        wp_buffer[len + 1 + 1] = cs2;
        wp_buffer[len + 1 + 2] = 0x69;
        uart_write(UART_NUM_0, wp_buffer, sizeof(wp_buffer));
    }
    else if (data[0] == 0xFC)
    {
        uint8_t motor_test_result_buffer[len + 1 + 3];
        motor_test_result_buffer[0] = 0xFC;
        motor_test_result_buffer[1] = len - 1 + 3;
        memcpy(motor_test_result_buffer + 2, data + 1, len - 1);
        uint8_t cs1, cs2;
        checksum_generate(motor_test_result_buffer + 2, len - 1, &cs1, &cs2);
        motor_test_result_buffer[len + 1 + 0] = cs1;
        motor_test_result_buffer[len + 1 + 1] = cs2;
        motor_test_result_buffer[len + 1 + 2] = 0x69;
        uart_write(UART_NUM_0, motor_test_result_buffer, sizeof(motor_test_result_buffer));
    }

    static uint8_t led_state = 0;
    if (led_state == 0)
    {
        gpio_set_level(GPIO_NUM_2, 1);
        led_state = 1;
    }
    else
    {
        gpio_set_level(GPIO_NUM_2, 0);
        led_state = 0;
    }
}
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}


static void checksum_generate(uint8_t *data, uint8_t size, uint8_t *cs1, uint8_t *cs2)
{
    uint8_t checksum1 = 0, checksum2 = 0;

    for (uint8_t i = 0; i < size; i++)
    {
        checksum1 = checksum1 + data[i];
        checksum2 = checksum2 + checksum1;
    }

    *cs1 = checksum1;
    *cs2 = checksum2;
}

static uint8_t checksum_verify(uint8_t *data, uint8_t size)
{
    uint8_t c1, c2;
    checksum_generate(data, size - 3, &c1, &c2);
    if (c1 == data[size - 3] && c2 == data[size - 2])
        return 1;
    return 0;
}