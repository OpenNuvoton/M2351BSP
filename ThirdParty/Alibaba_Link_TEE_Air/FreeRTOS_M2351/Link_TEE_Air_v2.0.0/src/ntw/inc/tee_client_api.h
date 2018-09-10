/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef _TEE_CLIENT_API_H_
#define _TEE_CLIENT_API_H_

#include "tee_types.h"

/*
 * API Error Codes
 */
#define TEEC_SUCCESS                   (0x00000000)
#define TEEC_ERROR_GENERIC             (0xFFFF0000)
#define TEEC_ERROR_ACCESS_DENIED       (0xFFFF0001)
#define TEEC_ERROR_CANCEL              (0xFFFF0002)
#define TEEC_ERROR_ACCESS_CONFLICT     (0xFFFF0003)
#define TEEC_ERROR_EXCESS_DATA         (0xFFFF0004)
#define TEEC_ERROR_BAD_FORMAT          (0xFFFF0005)
#define TEEC_ERROR_BAD_PARAMETERS      (0xFFFF0006)
#define TEEC_ERROR_BAD_STATE           (0xFFFF0007)
#define TEEC_ERROR_ITEM_NOT_FOUND      (0xFFFF0008)
#define TEEC_ERROR_NOT_IMPLEMENTED     (0xFFFF0009)
#define TEEC_ERROR_NOT_SUPPORTED       (0xFFFF000A)
#define TEEC_ERROR_NO_DATA             (0xFFFF000B)
#define TEEC_ERROR_OUT_OF_MEMORY       (0xFFFF000C)
#define TEEC_ERROR_BUSY                (0xFFFF000D)
#define TEEC_ERROR_COMMUNICATION       (0xFFFF000E)
#define TEEC_ERROR_SECURITY            (0xFFFF000F)
#define TEEC_ERROR_SHORT_BUFFER        (0xFFFF0010)
#define TEEC_ERROR_TARGET_DEAD         (0xFFFF3024)

#define TEEC_NONE                    (0x0)
#define TEEC_VALUE                   (0x1)
#define TEEC_PTR                     (0x2)

#define TEEC_PARAM_TYPES(t0, t1, t2, t3) \
        ((t0) | ((t1) << 4) | ((t2) << 8) | ((t3) << 12))

typedef uint32_t teec_stat_t;

typedef struct _teec_uuid_t
{
    uint32_t time_low;
    uint16_t time_mid;
    uint16_t time_hi_and_ver;
    uint8_t  clk_seq_and_node[8];
} teec_uuid_t;

typedef void * teec_ss_t;

typedef struct _teec_val_t
{
    uint32_t val_a;
    uint32_t val_b;
} teec_val_t;

typedef struct _teec_shm_t
{
    union
    {
        uintptr_t buf;
        uint64_t  rsvd;
    };
    uint32_t  size;
} teec_shm_t;

typedef union _teec_param_t
{
    teec_val_t val;
    teec_shm_t shm;
} teec_param_t;

typedef struct _teec_op_t
{
    uint32_t     param_types;
    teec_param_t params[4];
} teec_op_t;

teec_stat_t teec_open_ss(
        const teec_uuid_t *dst, teec_ss_t *ss, uint32_t *ret_orig);
void teec_close_ss(teec_ss_t ss);
teec_stat_t teec_inv_op(
        teec_ss_t ss, uint32_t cmd_id, teec_op_t *op, uint32_t *ret_orig);

uint32_t teec_fast_call(uint32_t id, void *arg, size_t len);
#endif /* _TEE_CLIENT_API_H_ */
