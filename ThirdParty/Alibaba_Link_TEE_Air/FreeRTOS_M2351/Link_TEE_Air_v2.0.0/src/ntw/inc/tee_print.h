/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef _TEE_PRINT_H_
#define _TEE_PRINT_H_

#include "tee_types.h"

#ifdef CONFIG_DBG

#include <stdio.h>
#define tee_print(_f, _a ...) printf(_f, ##_a)

#endif

#endif /* _TEE_PRINT_H_ */
