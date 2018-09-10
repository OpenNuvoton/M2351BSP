/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef _TEE_TYPES_H_
#define _TEE_TYPES_H_

#define _SSIZE_T_DECLARED

#include <stddef.h>
#include <stdint.h>

#ifndef __STDINT__

#ifndef _INT8_T_DECLARED
typedef signed char                int8_t;
#endif
typedef unsigned char       uint8_t;

typedef short               int16_t;
typedef unsigned short      uint16_t;

#ifndef _INT32_T_DECLARED
typedef int                 int32_t;
#endif
#ifndef _UINT32_T_DECLARED
typedef unsigned int        uint32_t;
#endif

typedef long long           int64_t;
typedef unsigned long long  uint64_t;

//typedef unsigned long       size_t;
typedef long                ssize_t;

typedef long                long_t;
typedef unsigned long       ulong_t;

//typedef unsigned long int   uintptr_t;

#endif /* __STDINT__ */

typedef unsigned char       bool;
typedef uint32_t            tee_stat_t;

#ifndef false
#define false               (0)
#endif

#ifndef true
#define true                (1)
#endif

#ifndef NULL
#define NULL                ((void *)0)
#endif

#endif /* _TEE_TYPES_H_ */
