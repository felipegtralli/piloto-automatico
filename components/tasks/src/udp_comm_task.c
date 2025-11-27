#include <stdint.h>
#include "lwip/arch.h"
#include "tasks.h"
#include "wifi.h"
#include "udp_private/udp_priv.h"
#include "error_handling.h"

void udp_comm_task(void* pvParameters) {
    udp_comm_task_ctx* ctx = pvParameters;

    int sock = udp_create_socket();
    if(sock < 0) {
        handle_comm_task_state(SYSTEM_STATE_COMM_TASK_FAIL, ctx, sock);
        return;
    }

    uint8_t rx_buffer[UDP_MAX_RX_BUF_SIZE] = {0};
    struct sockaddr_in src_addr = {0};
    for(;;) {
        if(!ctx->ehandler_ctx->station_connected) {
            udp_reset_dest_addr();
            ctx->first_exchange = true;
            handle_comm_task_state(SYSTEM_STATE_COMM_TASK_PAUSED, NULL, -1);
            vTaskSuspend(NULL);
        }
        ssize_t len = wifi_udp_receive(sock, rx_buffer, sizeof(rx_buffer) - 1, &src_addr, &ctx->first_exchange);

        if(len > 0) {
            uint8_t tx_buffer[UDP_MAX_TX_BUF_SIZE] = {0};
            size_t tx_len = udp_handle_command(rx_buffer, len, tx_buffer, sizeof(tx_buffer), ctx);

            (void)wifi_udp_send(sock, tx_buffer, tx_len);
        }
    }
}
