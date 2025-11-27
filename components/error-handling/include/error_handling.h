#ifndef __ERROR_HANDLING_H__
#define __ERROR_HANDLING_H__

#include "tasks.h"

typedef enum {
    SYSTEM_STATE_CONTROL_TASK_RUNNNING,
    SYSTEM_STATE_CONTROL_TASK_PAUSED,
    SYSTEM_STATE_CONTROL_TASK_FAIL,

    SYSTEM_STATE_COMM_TASK_RUNNING,
    SYSTEM_STATE_COMM_TASK_PAUSED,
    SYSTEM_STATE_COMM_TASK_FAIL,

    SYSTEM_STATE_ERROR,
} system_state;

void handle_control_task_state(system_state state, control_task_ctx* ctx);
void handle_comm_task_state(system_state state, udp_comm_task_ctx* ctx, int sock);

#endif // __ERROR_HANDLING_H__
