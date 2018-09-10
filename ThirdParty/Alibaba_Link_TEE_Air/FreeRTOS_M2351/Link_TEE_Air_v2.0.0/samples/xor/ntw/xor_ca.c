/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include "tee_client_api.h"
#include "tee_print.h"
#include "xor.h"

static const teec_uuid_t _g_uuid = XOR_SRV_UUID;

/* c = a ^ b */
int tee_xor_test()
{
    unsigned int     a = 0, b = 1, c = 0;
    teec_ss_t        ss;
    teec_op_t        op;
    teec_stat_t      ret = TEEC_SUCCESS;
    uint32_t         ret_orig;

    ret = teec_open_ss(&_g_uuid, &ss, &ret_orig);
    if (ret != TEEC_SUCCESS) {
        tee_print("failed to call TEEC_OpenSession, 0x%08x\n", ret);
        goto cleanup2;
    }

#if CONFIG_TEST_PERF
    op.param_types = TEEC_PARAM_TYPES(
            TEEC_NONE, TEEC_NONE,
            TEEC_NONE, TEEC_NONE);
    int round;
    extern volatile uint32_t ticks;
    volatile uint32_t ticks_before , ticks_after;
    ticks_before = ticks;
    tee_print("before tee xor test perf tick %d\n", ticks_before);

    round = 1000;
    while (round-- > 0) {
        ret = teec_inv_op(ss, CMD_XOR_PERF, &op, &ret_orig);
        if (ret != TEEC_SUCCESS) {
            tee_print("failed to call TEEC_InvokeCommand, 0x%08x\n", ret);
            goto cleanup3;
        }
    }
    ticks_after = ticks;
    tee_print("after tee xor test perf tick %d, totla time %d ms\n", ticks_after, (ticks_after - ticks_before) * 10);
#else
    op.param_types = TEEC_PARAM_TYPES(
            TEEC_VALUE, TEEC_VALUE,
            TEEC_VALUE, TEEC_NONE);
    op.params[0].val.val_a = a;
    op.params[1].val.val_a = b;
    op.params[2].val.val_a = c;
    ret = teec_inv_op(ss, CMD_XOR, &op, &ret_orig);
    if (ret != TEEC_SUCCESS) {
        tee_print("failed to call TEEC_InvokeCommand, 0x%08x\n", ret);
        goto cleanup3;
    }
    c = op.params[2].val.val_a;

    tee_print("a = %d; b = %d; c = a ^ b: %d\n", a, b, c);
#endif

cleanup3:
    teec_close_ss(ss);
cleanup2:

    return ret;
}
