#ifndef __WIFI_H__
#define __WIFI_H__

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_event_base.h"
#include "lwip/arch.h"
#include "lwip/sockets.h"

#define WIFI_AP_SSID "esp32-piloto-automatico"
#define WIFI_AP_PASSWORD "esp32@1234"
#define WIFI_AP_CHANNEL 1U
#define WIFI_AP_MAX_CONNECTIONS 1U

typedef struct {
    volatile bool station_connected;
} wifi_ap_event_handler_ctx;

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
int udp_create_socket(void);

ssize_t wifi_udp_receive(const int sock, uint8_t* rx_buffer, size_t rx_buffer_len, struct sockaddr_in* src_addr, bool* first_exchange);
ssize_t wifi_udp_send(const int sock, uint8_t* tx_buffer, size_t tx_buffer_len);

#endif // __WIFI_H__
