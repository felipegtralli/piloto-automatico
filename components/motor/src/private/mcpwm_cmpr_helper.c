#include "driver/mcpwm_types.h"
#include "mcpwm_private.h"

int mcpwm_get_operator_id(mcpwm_cmpr_handle_t cmpr) {
    if(cmpr == NULL) {
        return -1;
    }

    mcpwm_oper_t* oper = cmpr->oper;
    if(oper == NULL) {
        return -1;
    }

    return oper->oper_id;
}

int mcpwm_get_comparator_id(mcpwm_cmpr_handle_t cmpr) {
    if(cmpr == NULL) {
        return -1;
    }

    return cmpr->cmpr_id;
}
