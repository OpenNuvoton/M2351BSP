/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include "tee_tos.h"
#include "tee_srv.h"
#include "xor.h"

static TEE_Result _xor_CreateEntryPoint(void)
{
    return TEE_SUCCESS;
}

static void _xor_DestroyEntryPoint(void)
{
    return;
}

static TEE_Result _xor_OpenSessionEntryPoint(
        void **sessionContext)
{
    return TEE_SUCCESS;
}

static void _xor_CloseSessionEntryPoint(void *sessionContext)
{
    return;
}

static TEE_Result _xor_InvokeCommandEntryPoint(
        void *sessionContext,
        uint32_t commandID,
        uint32_t paramTypes,
        tee_param_t params[4])
{
#if CONFIG_TEST_PERF
    if (paramTypes != TEE_PARAM_TYPES(
                TEE_NONE,
                TEE_NONE,
                TEE_NONE,
                TEE_NONE)) {
        return TEE_ERROR_BAD_PARAMETERS;
    }

    if (CMD_XOR_PERF != commandID) {
        return TEE_ERROR_BAD_PARAMETERS;
    }

    return TEE_SUCCESS;
#else
    if (paramTypes != TEE_PARAM_TYPES(
                TEE_VALUE,
                TEE_VALUE,
                TEE_VALUE,
                TEE_NONE)) {
        return TEE_ERROR_BAD_PARAMETERS;
    }

    if (CMD_XOR != commandID) {
        return TEE_ERROR_BAD_PARAMETERS;
    }

    params[2].val.val_a =
        params[0].val.val_a ^ params[1].val.val_a;

    tee_print("xor: a 0x%08x, b 0x%08x, xor 0x%08x\n",
        params[0].val.val_a, params[1].val.val_a, params[2].val.val_a);

#endif

    return TEE_SUCCESS;
}

TEE_SRV_DATA_START(_xor_CreateEntryPoint,
        _xor_DestroyEntryPoint,
        _xor_OpenSessionEntryPoint,
        _xor_CloseSessionEntryPoint,
        _xor_InvokeCommandEntryPoint)
TEE_SRV_UUID_PROPERTY("gpd.ta.appID", XOR_SRV_UUID)
TEE_SRV_BOOL_PROPERTY("gpd.ta.singleInstance", true)
TEE_SRV_BOOL_PROPERTY("gpd.ta.multiSession", true)
TEE_SRV_BOOL_PROPERTY("gpd.ta.instanceKeepAlive", true)
TEE_SRV_INT_PROPERTY("gpd.ta.dataSize", 0x1000)
TEE_SRV_INT_PROPERTY("gpd.ta.stackSize", 0x800)
TEE_SRV_DATA_END
