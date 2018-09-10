/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef _TEE_OSA_PRINT_H_
#define _TEE_OSA_PRINT_H_

#include "tee_types.h"
#include <stdio.h>

#define tee_osa_print(_f, _a ...) printf(_f, ##_a)

void tee_osa_backtrace(void);

#endif /* _TEE_OSA_PRINT_H_ */
