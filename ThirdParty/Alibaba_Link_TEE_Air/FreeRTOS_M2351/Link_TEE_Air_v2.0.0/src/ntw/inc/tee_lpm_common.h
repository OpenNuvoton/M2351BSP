/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef _TEE_LPM_COMMON_H_
#define _TEE_LPM_COMMON_H_

#include "tee_types.h"

typedef enum  _lpm_mode_t
{
    LPM_IDLE = 0,
    LPM_PD,
    LPM_FWPD,
    LPM_LLPD,
    LPM_SPD,
    LPM_DPD,
} lpm_mode_t;

typedef struct _lpm_args_t
{
    uint32_t mode;
    uint32_t param0;
    uint32_t param1;
    uint32_t param2;
    uint32_t param3;
    uint32_t rsvd;
} lpm_args_t;

#endif /* _TEE_LPM_COMMON_H_ */
