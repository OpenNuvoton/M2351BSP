/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef _TEE_OSA_SYNC_H_
#define _TEE_OSA_SYNC_H_

#include "tee_types.h"

#define TEE_OSA_TIME_INFINITE   (-1)

typedef void * tee_osa_mutex_t;

tee_osa_mutex_t tee_osa_create_mutex(void);
void tee_osa_destroy_mutex(tee_osa_mutex_t mutex);

tee_stat_t tee_osa_wait_for_mutex_timeout(tee_osa_mutex_t mutex, int32_t msec);
void tee_osa_release_mutex(tee_osa_mutex_t mutex);

#endif /* _TEE_OSA_SYNC_H_ */

