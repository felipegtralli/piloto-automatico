#ifndef __UDP_PRIV_H__
#define __UDP_PRIV_H__

#include <stdint.h>
#include <stddef.h>
#include "tasks.h"

/*
 rx command format:
    [command (1 byte)] [flags (1 byte)] [data (N bytes)]

 tx command format:
    [command (1 byte)] [data (N bytes)]
*/

#define UDP_MAX_RX_BUF_SIZE 20U
#define UDP_MAX_TX_BUF_SIZE 16U

typedef enum {
    // get
    CMD_GET_BASE = 0x00,
    CMD_GET_RPM = CMD_GET_BASE + 0x01,
    CMD_GET_SETPOINT = CMD_GET_BASE + 0x02,
    CMD_GET_PID_GAINS = CMD_GET_BASE + 0x03,
    CMD_GET_PID_ANTI_WINDUP = CMD_GET_BASE + 0x04,
    CMD_GET_PID_OUTPUT_LIMITS = CMD_GET_BASE + 0x05,
    CMD_GET_LPF_STATE = CMD_GET_BASE + 0x06,

    // set
    CMD_SET_BASE = 0x80,
    CMD_SET_SETPOINT = CMD_SET_BASE + 0x01,
    CMD_SET_PID_GAINS = CMD_SET_BASE + 0x02,
    CMD_SET_PID_ANTI_WINDUP = CMD_SET_BASE + 0x03,
    CMD_SET_PID_OUTPUT_LIMITS = CMD_SET_BASE + 0x04,

    CMD_UNKNOWN = 0xFF
} udp_command;

size_t udp_handle_command(const uint8_t* rx_buffer, size_t len, uint8_t* tx_buffer, size_t tx_max_len, udp_comm_task_ctx* udp_ctx);

#endif // __UDP_PRIV_H__