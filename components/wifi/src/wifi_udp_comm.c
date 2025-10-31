#include "freertos/FreeRTOS.h"
#include "lwip/arch.h"
#include "lwip/sockets.h"
#include "wifi.h"
#include <stdint.h>

#define UDP_CLIENT_PORT 12345

static struct sockaddr_in dest_addr = {
    .sin_addr.s_addr = 0,
    .sin_family = AF_INET,
    .sin_port = htons(UDP_CLIENT_PORT),
};

int udp_create_socket(void) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);  

    if(sock > 0) {
        struct sockaddr_in local_addr = {
            .sin_family = AF_INET,
            .sin_port = htons(UDP_CLIENT_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY)
        };

        int ret = bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr));
        if(ret < 0) {
            closesocket(sock);
            sock = -1;
        }
    }
    return sock;  
}

ssize_t wifi_udp_receive(const int sock, uint8_t* rx_buffer, size_t rx_buffer_len, struct sockaddr_in* src_addr, bool* first_exchange) {
    socklen_t src_addr_len = sizeof(struct sockaddr_in);

    ssize_t len = recvfrom(sock, rx_buffer, rx_buffer_len, 0, (struct sockaddr*)src_addr, &src_addr_len);
    if(len > 0) {
        if(*first_exchange) {
            dest_addr.sin_addr.s_addr = src_addr->sin_addr.s_addr;
            *first_exchange = false;
        }

        rx_buffer[len] = 0; 
    }

    return len;
}

ssize_t wifi_udp_send(const int sock, uint8_t* tx_buffer, size_t tx_buffer_len) {
    ssize_t sent = 0;

    if(dest_addr.sin_addr.s_addr != 0) {
        sent = sendto(sock, tx_buffer, tx_buffer_len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    } else {
        sent = -1;
    }

    return sent;
}
